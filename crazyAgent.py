import os
from dotenv import load_dotenv
import gradio as gr
from langgraph.graph import StateGraph, END
from langgraph.prebuilt import ToolNode, tools_condition
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.messages import HumanMessage, AIMessage, BaseMessage
from typing import TypedDict, List, Annotated
from langgraph.graph.message import add_messages
from tools import google_search_tool, drone_takeoff_tool, drone_land_tool


load_dotenv()


class AgentState(TypedDict):
    messages: Annotated[List[BaseMessage], add_messages]


class CrazyAgent:
    def __init__(self):
        self.llm = ChatGoogleGenerativeAI(
            model="gemini-2.5-flash",
            google_api_key=os.getenv("GOOGLE_API_KEY"),
            temperature=0.7
        )
        
        self.tools = [google_search_tool, drone_takeoff_tool, drone_land_tool]
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
        response = self.llm_with_tools.invoke(messages)
        return {"messages": [response]}
    
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
        description="An AI agent powered by Gemini Flash 2.5 with Google Search and Crazyflie drone control capabilities",
        examples=[
            "What's the latest news about AI?",
            "Launch the drone",
            "Land the drone",
            "Search for information about Python programming"
        ]
    )
    
    interface.launch(share=True)


if __name__ == "__main__":
    main()