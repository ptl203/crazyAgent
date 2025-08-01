import os
import logging
from dotenv import load_dotenv
import gradio as gr
from langgraph.graph import StateGraph, END
from langgraph.prebuilt import ToolNode, tools_condition
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.messages import HumanMessage, AIMessage, BaseMessage
from typing import TypedDict, List, Annotated
from langgraph.graph.message import add_messages
from tools import drone_takeoff_tool, drone_land_tool, drone_goto_tool, drone_turn_tool

load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('crazyagent.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class AgentState(TypedDict):
    messages: Annotated[List[BaseMessage], add_messages]


class CrazyAgent:
    def __init__(self):
        self.llm = ChatGoogleGenerativeAI(
            model="gemini-2.5-flash",
            google_api_key=os.getenv("GOOGLE_API_KEY"),
            temperature=0.7
        )
        
        self.tools = [drone_takeoff_tool, drone_land_tool, drone_goto_tool, drone_turn_tool]
        self.llm_with_tools = self.llm.bind_tools(self.tools)
        self.tool_node = ToolNode(self.tools)
        
        self.graph = self._create_graph()
    
    def _create_graph(self):
        workflow = StateGraph(AgentState)
        
        workflow.add_node("agent", self._call_model)
        workflow.add_node("tools", self.tool_node)
        
        workflow.set_entry_point("agent")
        
        workflow.add_conditional_edges(
            "agent",
            tools_condition,
        )
        
        workflow.add_edge("tools", "agent")
        
        return workflow.compile()
    
    def _call_model(self, state: AgentState):
        messages = state["messages"]
        logger.info(f"Calling model with {len(messages)} messages")
        
        # Log the last message (user input)
        if messages:
            last_msg = messages[-1]
            logger.info(f"Last message type: {type(last_msg).__name__}, content: {last_msg.content}")
        
        response = self.llm_with_tools.invoke(messages)
        
        # Log if the response contains tool calls
        if hasattr(response, 'tool_calls') and response.tool_calls:
            logger.info(f"Model response contains {len(response.tool_calls)} tool calls")
            for i, tool_call in enumerate(response.tool_calls):
                logger.info(f"Tool call {i+1}: name='{tool_call['name']}'")
                args = tool_call.get('args', {})
                logger.info(f"  Raw args dict: {args}")
                logger.info(f"  Args type: {type(args).__name__}")
                for key, value in args.items():
                    logger.info(f"    {key}: {repr(value)} (type: {type(value).__name__})")
        else:
            logger.info("Model response contains no tool calls")
            logger.info(f"Response content: {response.content}")
        
        return {"messages": [response]}
    
    def chat(self, message: str, history: List[List[str]]):
        logger.info(f"=== NEW CHAT SESSION ===")
        logger.info(f"User message: {message}")
        logger.info(f"History length: {len(history)}")
        
        messages = []
        
        for human, ai in history:
            messages.append(HumanMessage(content=human))
            messages.append(AIMessage(content=ai))
        
        messages.append(HumanMessage(content=message))
        
        logger.info(f"Invoking graph with {len(messages)} total messages")
        
        try:
            result = self.graph.invoke({"messages": messages})
            logger.info(f"Graph execution completed. Result messages: {len(result['messages'])}")
            
            # Log all messages in the result
            for i, msg in enumerate(result["messages"]):
                logger.info(f"Result message {i+1}: type={type(msg).__name__}")
                if hasattr(msg, 'tool_calls') and msg.tool_calls:
                    logger.info(f"  - Contains {len(msg.tool_calls)} tool calls")
                if hasattr(msg, 'content'):
                    logger.info(f"  - Content: {msg.content}")
            
            response = result["messages"][-1].content
            logger.info(f"Final response: {response}")
            return response
            
        except Exception as e:
            logger.error(f"Error during graph execution: {str(e)}", exc_info=True)
            return f"Error: {str(e)}"


def main():
    agent = CrazyAgent()
    
    def chat_interface(message, history):
        return agent.chat(message, history)
    
    interface = gr.ChatInterface(
        fn=chat_interface,
        title="🤖 Crazy Agent",
        description="An AI agent powered by Gemini Flash 2.5 with Google Search and Crazyflie drone control capabilities",
        #examples=[
        #    "What's the latest news about AI?",
        #    "Launch the drone",
        #    "Land the drone",
        #    "Search for information about Python programming"
        #]
    )
    
    interface.launch(share=True)


if __name__ == "__main__":
    main()