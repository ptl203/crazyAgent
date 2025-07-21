from langchain_google_community import GoogleSearchAPIWrapper
from langchain.tools import Tool
import os

#Load Environment
from dotenv import load_dotenv

load_dotenv()

def create_google_search_tool():
    """Create a Google Search tool for the AI agent."""
    search = GoogleSearchAPIWrapper()
    
    return Tool(
        name="google_search",
        description="Search Google for current information. Use this when you need to find recent information or facts that you might not have in your knowledge base.",
        func=search.run
    )