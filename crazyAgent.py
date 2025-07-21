import os
from dotenv import load_dotenv
import gradio as gr
from langgraph.graph import StateGraph, END
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain.schema import HumanMessage, AIMessage
from langchain_core.messages import ToolMessage
from typing import TypedDict, List, Union
from tools import create_google_search_tool

load_dotenv()


class AgentState(TypedDict):
    messages: List[Union[HumanMessage, AIMessage]]


class CrazyAgent:
    def __init__(self):
        self.llm = ChatGoogleGenerativeAI(
            model="gemini-2.0-flash-exp",
            google_api_key=os.getenv("GOOGLE_API_KEY"),
            temperature=0.7
        )
        
        self.tools = [create_google_search_tool()]
        self.tools_by_name = {tool.name: tool for tool in self.tools}
        self.llm_with_tools = self.llm.bind_tools(self.tools)
        
        self.graph = self._create_graph()
    
    def _create_graph(self):
        workflow = StateGraph(AgentState)
        
        workflow.add_node("agent", self._call_model)
        workflow.add_node("action", self._call_tool)
        
        workflow.set_entry_point("agent")
        
        workflow.add_conditional_edges(
            "agent",
            self._should_continue,
            {
                "continue": "action",
                "end": END
            }
        )
        
        workflow.add_edge("action", "agent")
        
        return workflow.compile()
    
    def _call_model(self, state: AgentState):
        messages = state["messages"]
        response = self.llm_with_tools.invoke(messages)
        return {"messages": messages + [response]}
    
    def _call_tool(self, state: AgentState):
        messages = state["messages"]
        last_message = messages[-1]
        
        tool_calls = last_message.tool_calls
        for tool_call in tool_calls:
            tool_name = tool_call["name"]
            tool_args = tool_call["args"]
            
            if tool_name in self.tools_by_name:
                tool = self.tools_by_name[tool_name]
                tool_result = tool.func(**tool_args)
                
                tool_message = ToolMessage(
                    content=str(tool_result),
                    tool_call_id=tool_call["id"]
                )
                messages.append(tool_message)
        
        return {"messages": messages}
    
    def _should_continue(self, state: AgentState):
        messages = state["messages"]
        last_message = messages[-1]
        
        if hasattr(last_message, 'tool_calls') and last_message.tool_calls:
            return "continue"
        else:
            return "end"
    
    def chat(self, message: str, history: List[List[str]]):
        messages = []
        
        for human, ai in history:
            messages.append(HumanMessage(content=human))
            messages.append(AIMessage(content=ai))
        
        messages.append(HumanMessage(content=message))
        
        result = self.graph.invoke({"messages": messages})
        
        response = result["messages"][-1].content
        return response


def main():
    agent = CrazyAgent()
    
    def chat_interface(message, history):
        return agent.chat(message, history)
    
    interface = gr.ChatInterface(
        fn=chat_interface,
        title="🤖 Crazy Agent",
        description="An AI agent powered by Gemini Flash 2.5 with Google Search capabilities",
        examples=[
            "What's the latest news about AI?",
            "Search for information about Python programming",
            "What's the weather like today?"
        ]
    )
    
    interface.launch(share=True)


if __name__ == "__main__":
    main()