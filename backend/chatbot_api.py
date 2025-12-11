"""
FastAPI server for RAG Chatbot
Exposes REST API endpoints for chatbot interactions
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
from dotenv import load_dotenv
from rag_chatbot import RAGChatbot, get_chatbot

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="Physical AI RAG Chatbot API",
    description="RAG Chatbot using Gemini 2.0 Flash for the Physical AI & Robotics Book",
    version="1.0.0",
)

# Add CORS middleware for frontend access
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "*").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize chatbot
chatbot: Optional[RAGChatbot] = None


def get_initialized_chatbot() -> RAGChatbot:
    """Get initialized chatbot instance."""
    global chatbot
    if chatbot is None:
        chatbot = get_chatbot()
    return chatbot


# ========================
# Pydantic Models
# ========================


class MessageModel(BaseModel):
    """Single message in conversation."""

    role: str  # "user" or "assistant"
    content: str


class ChatRequestModel(BaseModel):
    """Request model for chat endpoint."""

    message: str
    conversation_history: Optional[List[MessageModel]] = None


class ChatResponseModel(BaseModel):
    """Response model for chat endpoint."""

    response: str
    sources: Optional[List[dict]] = None
    status: str = "success"


class SystemInfoModel(BaseModel):
    """System information about the chatbot."""

    status: str
    model: str
    collection: str
    documents_count: Optional[int] = None
    vector_size: Optional[int] = None


# ========================
# API Endpoints
# ========================


@app.get("/", tags=["Health"])
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": {
            "chat": "/api/chat",
            "health": "/api/health",
            "info": "/api/info",
        },
    }


@app.get("/api/health", tags=["Health"])
async def health():
    """Health check endpoint."""
    try:
        chatbot = get_initialized_chatbot()
        info = chatbot.get_system_info()
        return {
            "status": info.get("status", "unknown"),
            "message": "Chatbot is operational",
        }
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"Service unavailable: {str(e)}")


@app.get("/api/info", response_model=SystemInfoModel, tags=["Information"])
async def get_info():
    """Get chatbot and knowledge base information."""
    try:
        chatbot = get_initialized_chatbot()
        info = chatbot.get_system_info()
        return SystemInfoModel(**info)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving info: {str(e)}")


@app.post("/api/chat", response_model=ChatResponseModel, tags=["Chat"])
async def chat(request: ChatRequestModel):
    """
    Chat with the RAG chatbot.

    Args:
        message: User's question or message
        conversation_history: Previous messages in conversation (optional)

    Returns:
        Generated response from Gemini based on book knowledge
    """
    try:
        if not request.message or request.message.strip() == "":
            raise HTTPException(status_code=400, detail="Message cannot be empty")

        chatbot = get_initialized_chatbot()

        # Convert conversation history if provided
        history = []
        if request.conversation_history:
            for msg in request.conversation_history:
                history.append({"role": msg.role, "content": msg.content})

        # Generate response
        response_text = chatbot.chat(request.message, history)

        return ChatResponseModel(
            response=response_text,
            status="success",
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error processing chat request: {str(e)}",
        )


@app.post("/api/stream", tags=["Chat"])
async def stream_chat(request: ChatRequestModel):
    """
    Stream chat response (placeholder for future streaming implementation).
    Currently returns full response.
    """
    try:
        chatbot = get_initialized_chatbot()
        response_text = chatbot.chat(request.message)
        return {"response": response_text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


# ========================
# Startup/Shutdown
# ========================


@app.on_event("startup")
async def startup_event():
    """Initialize chatbot on startup."""
    print("🚀 Starting RAG Chatbot API...")
    try:
        chatbot = get_initialized_chatbot()
        info = chatbot.get_system_info()
        print(f"✅ Chatbot initialized successfully")
        print(f"   Model: {info.get('model')}")
        print(f"   Collection: {info.get('collection')}")
        print(f"   Documents: {info.get('documents_count', 'N/A')}")
    except Exception as e:
        print(f"⚠️  Warning during startup: {e}")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    print("🛑 Shutting down RAG Chatbot API...")


# ========================
# Error Handlers
# ========================


@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    """Custom HTTP exception handler."""
    return {
        "status": "error",
        "message": exc.detail,
        "status_code": exc.status_code,
    }


if __name__ == "__main__":
    import uvicorn

    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")

    print(f"🌍 Starting server on {host}:{port}")
    uvicorn.run(app, host=host, port=port)
