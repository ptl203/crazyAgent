import os
import re
import json
import difflib
import logging
from dotenv import load_dotenv
import gradio as gr

from typing import TypedDict, List, Annotated, Dict, Any

from langgraph.graph import StateGraph, END
from langgraph.graph.message import add_messages

from langchain_core.messages import (
    SystemMessage,
    HumanMessage,
    AIMessage,
    BaseMessage,
)

# Newer LangChain:
from langchain_ollama import ChatOllama
# (If you're on the older community split, switch to:
# from langchain_community.chat_models import ChatOllama)

# === Your tools ===
from tools import (
    drone_takeoff_tool,
    drone_land_tool,
    drone_goto_tool,
    drone_turn_tool,
)
from retriever import get_objective_coordinates


load_dotenv()

# =========================
# Logging
# =========================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.FileHandler("crazyagent.log"), logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

# =========================
# ReAct system prompt (multi-step explicit)
# =========================
REACT_SYSTEM_PROMPT = """You are a concise, helpful agent that controls a Crazyflie drone using TOOLS.

CRITICAL RULES:
1) Prefer calling a tool over guessing whenever a tool can achieve the user's goal.
2) When calling a tool, output EXACTLY this format (no other text before/after):
Action: <tool_name>
Action Input: <JSON object>
3) You may need MULTIPLE tool calls across several steps. After each Observation, continue planning and either call another tool or finalize.
4) When the user's goal is satisfied, output exactly:
Final: Action Completed
5) Keep natural-language replies short.

Available tools and their arguments (JSON keys):
- drone_takeoff_tool        { "height_m": number }
- drone_land_tool           { }   # no arguments
- drone_goto_tool           { "x": number, "y": number, "z": number }
- drone_turn_tool           { "yaw_deg": number }
- get_objective_coordinates { "objective": string }

Use the tool names EXACTLY as listed (case-insensitive; underscores allowed).

Examples:
User: take off to 1.2 meters
Action: drone_takeoff_tool
Action Input: {"height_m": 1.2}

User: land now
Action: drone_land_tool
Action Input: {}

User: go to x=1.0 y=2.0 z=0.8
Action: drone_goto_tool
Action Input: {"x": 1.0, "y": 2.0, "z": 0.8}

User: turn 45 degrees
Action: drone_turn_tool
Action Input: {"yaw_deg": 45}

User: what's the plan?
Final: Action Completed
"""

# =========================
# State with a step counter + did_tool flag
# =========================
class AgentState(TypedDict):
    messages: Annotated[List[BaseMessage], add_messages]
    steps: int          # incremented on each node pass
    did_tool: bool      # set True once any tool ran this turn


# =========================
# Parse helpers for Action / Final
# =========================
ACTION_RE = re.compile(
    r"Action:\s*(?P<name>[A-Za-z0-9_\-\s]+)\s*[\r\n]+Action Input:\s*(?P<json>\{.*\})",
    re.DOTALL,
)
FINAL_RE = re.compile(r"^Final:\s*(?P<final>.+)$", re.DOTALL)

def parse_action_or_final(text: str) -> Dict[str, Any]:
    text = (text or "").strip()
    m_final = FINAL_RE.search(text)
    if m_final:
        return {"type": "final", "answer": m_final.group("final").strip()}

    m = ACTION_RE.search(text)
    if m:
        name = m.group("name").strip()
        json_str = m.group("json").strip()
        try:
            args = json.loads(json_str) if json_str else {}
        except json.JSONDecodeError:
            # try to recover common trailing-comma issues
            try:
                args = json.loads(re.sub(r",\s*}", "}", json_str))
            except Exception:
                args = {"_raw": json_str, "_error": "json_decode_failed"}
        return {"type": "action", "name": name, "args": args}

    return {"type": "none"}


# =========================
# Tool name resolver (case/space/dash tolerant + fuzzy)
# =========================
def resolve_tool_name(name: str, tools: Dict[str, Any]) -> str | None:
    """
    Normalize and fuzzy-match a tool name to one in `tools`.
    Accepts case-insensitive, spaces/dashes vs underscores, minor typos.
    """
    if not name:
        return None
    # normalize
    norm = name.strip().lower().replace("-", "_").replace(" ", "_")
    norm = re.sub(r"[^a-z0-9_]", "", norm)

    # exact match?
    if norm in tools:
        return norm

    # close match (e.g., 'drone_takeofftool' / 'Drone takeoff tool')
    candidates = difflib.get_close_matches(norm, list(tools.keys()), n=1, cutoff=0.7)
    return candidates[0] if candidates else None


# =========================
# Coerce tool input to match LangChain tool signature
# =========================
def coerce_tool_input(tool: Any, args: Any):
    """
    Map model-supplied args to what the LangChain tool expects.
    - If the tool expects 0 args: return ""
    - If it expects 1 arg: return that single value (or "" if empty)
    - If it expects >1 args: return the dict
    """
    try:
        spec: Dict[str, Any] = getattr(tool, "args", {}) or {}
        names = list(spec.keys())
    except Exception:
        # fallback: try to pass args directly
        return args

    # No-arg tool
    if len(names) == 0:
        return ""

    # Single-arg tool
    if len(names) == 1:
        name = names[0]
        if isinstance(args, dict):
            return args.get(name, "") if len(args) <= 1 else args.get(name, "")
        return args if args not in (None, {}, []) else ""

    # Multi-arg tool
    if isinstance(args, dict):
        return args
    return {"input": args}


# =========================
# Agent (ReAct over Ollama)
# =========================
class CrazyAgent:
    def __init__(self):
        # Config
        self.max_steps = 8  # cap the number of agent/tool cycles per user prompt

        # TinyLlama on your Pi via Ollama
        self.llm = ChatOllama(
            base_url="http://192.168.44.2:11434",
            model="tinyllama",
            temperature=0.35,                   # slightly lower to reduce waffle
            model_kwargs={"num_predict": 128},  # room for multi-step
        )

        # Available tools (as a dict for easy lookup)
        self.tools = {
            "drone_takeoff_tool": drone_takeoff_tool,
            "drone_land_tool": drone_land_tool,
            "drone_goto_tool": drone_goto_tool,
            "drone_turn_tool": drone_turn_tool,
            "get_objective_coordinates": get_objective_coordinates,
        }

        self.graph = self._create_graph()

    # ---- Build a small LangGraph: agent -> (maybe tool) -> agent ... until Final/limit ----
    def _create_graph(self):
        workflow = StateGraph(AgentState)

        workflow.add_node("agent", self._agent_node)
        workflow.add_node("tool", self._tool_node)

        workflow.set_entry_point("agent")
        workflow.add_conditional_edges("agent", self._route_from_agent)
        workflow.add_edge("tool", "agent")

        return workflow.compile()

    # ---- Nodes ----
    def _agent_node(self, state: AgentState) -> Dict[str, Any]:
        messages = state["messages"]
        steps = state.get("steps", 0)

        logger.info(f"[agent] steps={steps} messages={len(messages)}")

        # Prepend system prompt once
        if not messages or not isinstance(messages[0], SystemMessage):
            messages = [SystemMessage(content=REACT_SYSTEM_PROMPT)] + messages

        ai = self.llm.invoke(messages)
        logger.info(f"[agent] output:\n{getattr(ai, 'content', '')}")

        return {
            "messages": [ai],
            "steps": steps + 1,
            "did_tool": state.get("did_tool", False),
        }

    def _tool_node(self, state: AgentState) -> Dict[str, Any]:
        messages = state["messages"]
        steps = state.get("steps", 0)

        last_ai = next((m for m in reversed(messages) if isinstance(m, AIMessage)), None)
        if not last_ai:
            logger.warning("[tool] no AIMessage found")
            return {"messages": [], "steps": steps, "did_tool": state.get("did_tool", False)}

        parsed = parse_action_or_final(last_ai.content)
        if parsed.get("type") != "action":
            logger.warning("[tool] no Action parsed")
            return {"messages": [], "steps": steps, "did_tool": state.get("did_tool", False)}

        tool_name_raw = parsed["name"]
        args = parsed.get("args", {})

        resolved = resolve_tool_name(tool_name_raw, self.tools)
        tool = self.tools.get(resolved) if resolved else None

        if not tool:
            observation = (
                f"Error: unknown tool '{tool_name_raw}'. "
                f"Available (use these names, case-insensitive): {list(self.tools.keys())}"
            )
            logger.error(f"[tool] {observation}")
        else:
            try:
                tool_input = coerce_tool_input(tool, args)
                result = tool.invoke(tool_input)
                observation = f"Tool '{resolved}' result: {result}"
                logger.info(f"[tool] {observation}")
            except Exception as e:
                observation = f"Tool '{resolved}' error: {e}"
                logger.exception(f"[tool] {observation}")

        # IMPORTANT: Observation is a HumanMessage so the model "acts again"
        obs_msg = HumanMessage(
            content=(
                f"Observation: {observation}\n"
                "You must now EITHER:\n"
                "- Output another Action (with JSON) if more steps are needed, OR\n"
                "- Output Final: Action Completed if the goal is satisfied."
            )
        )
        return {"messages": [obs_msg], "steps": steps + 1, "did_tool": True}

    # ---- Router ----
    def _route_from_agent(self, state: AgentState) -> str:
        steps = state.get("steps", 0)
        if steps >= self.max_steps:
            logger.info(f"[router] reached step cap ({steps}) → END")
            return END

        messages = state["messages"]
        last_ai = next((m for m in reversed(messages) if isinstance(m, AIMessage)), None)
        if not last_ai:
            return END

        parsed = parse_action_or_final(last_ai.content or "")
        if parsed["type"] == "final":
            logger.info("[router] Final detected → END")
            return END
        if parsed["type"] == "action":
            logger.info(f"[router] Action detected → tool ({parsed.get('name')})")
            return "tool"

        logger.info("[router] No action/final → END")
        return END

    # ---- Public chat API for Gradio ----
    def chat(self, message: str, history: List[List[str]]) -> str:
        logger.info("=== NEW CHAT TURN ===")
        logger.info(f"User message: {message}")
        logger.info(f"History len: {len(history)}")

        msgs: List[BaseMessage] = []
        for human, ai in history:
            msgs.append(HumanMessage(content=human))
            msgs.append(AIMessage(content=ai))
        msgs.append(HumanMessage(content=message))

        try:
            # Start with steps=0, did_tool=False; the graph updates them
            result = self.graph.invoke({"messages": msgs, "steps": 0, "did_tool": False})

            # If the model produced a Final OR any tool ran → always return "Action Completed"
            did_tool = bool(result.get("did_tool"))
            final_detected = False
            for m in reversed(result["messages"]):
                if isinstance(m, AIMessage) and m.content:
                    parsed = parse_action_or_final(m.content)
                    if parsed["type"] == "final":
                        final_detected = True
                        break

            if did_tool or final_detected:
                return "Action Completed"

            # Fallback (should rarely happen)
            return "Action Completed"
        except Exception as e:
            logger.exception("Graph execution failed")
            return f"Error: {e}"


# =========================
# Gradio UI
# =========================
def main():
    agent = CrazyAgent()

    def chat_interface(message, history):
        return agent.chat(message, history)

    interface = gr.ChatInterface(
        fn=chat_interface,
        title="🤖 Crazy Agent",
        description="TinyLlama (Ollama on Raspberry Pi) controlling Crazyflie via ReAct tools.",
    )

    interface.launch(share=True)


if __name__ == "__main__":
    main()
