"""
RAG Chatbot using Gemini 2.0 Flash, Qdrant, and Cohere Embeddings
Retrieves relevant context from the Physical AI & Robotics Book and generates responses
"""

import os
from typing import Optional
import cohere
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "physical-ai-book")
EMBED_MODEL = os.getenv("EMBED_MODEL", "embed-english-v3.0")

# Validate required environment variables
if not COHERE_API_KEY:
    raise RuntimeError("Missing COHERE_API_KEY environment variable")
if not GEMINI_API_KEY:
    raise RuntimeError("Missing GEMINI_API_KEY environment variable")
if not QDRANT_URL or not QDRANT_API_KEY:
    raise RuntimeError("Missing QDRANT_URL or QDRANT_API_KEY environment variables")

# Initialize clients
cohere_client = cohere.Client(COHERE_API_KEY)
genai.configure(api_key=GEMINI_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)


class RAGChatbot:
    """
    Retrieval-Augmented Generation Chatbot
    Uses Cohere for embeddings, Qdrant for retrieval, Gemini for generation
    """

    def __init__(
        self,
        collection_name: str = COLLECTION_NAME,
        top_k: int = 5,
        model_name: str = "gemini-2.0-flash",
    ):
        """
        Initialize the RAG chatbot.

        Args:
            collection_name: Name of Qdrant collection with embedded documents
            top_k: Number of relevant documents to retrieve
            model_name: Gemini model to use for generation
        """
        self.collection_name = collection_name
        self.top_k = top_k
        self.model_name = model_name
        self.cohere_client = cohere_client
        self.qdrant = qdrant
        self.genai_model = genai.GenerativeModel(model_name)

    def _embed_query(self, query: str) -> list[float]:
        """
        Embed user query using Cohere.

        Args:
            query: User question/query

        Returns:
            Embedding vector
        """
        response = self.cohere_client.embed(
            model=EMBED_MODEL,
            texts=[query],
            input_type="search_query",
        )
        return response.embeddings[0]

    def _retrieve_context(self, query_embedding: list[float]) -> list[dict]:
        """
        Retrieve relevant documents from Qdrant.

        Args:
            query_embedding: Embedded query vector

        Returns:
            List of relevant documents with scores
        """
        try:
            results = self.qdrant.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=self.top_k,
                score_threshold=0.5,  # Only return documents with relevance > 0.5
            )

            context_docs = []
            for result in results:
                payload = result.payload
                context_docs.append(
                    {
                        "text": payload.get("text", ""),
                        "source": payload.get("source", "Unknown"),
                        "score": result.score,
                    }
                )

            return context_docs

        except Exception as e:
            print(f"[RAG] Error retrieving context from Qdrant: {e}")
            return []

    def _format_context(self, docs: list[dict]) -> str:
        """
        Format retrieved documents into a context string for the prompt.

        Args:
            docs: List of retrieved documents

        Returns:
            Formatted context string
        """
        if not docs:
            return "No relevant documents found in the knowledge base."

        context = "## Retrieved Context:\n\n"
        for i, doc in enumerate(docs, 1):
            context += f"**Source {i}** ({doc['source']}, relevance: {doc['score']:.2%})\n"
            context += f"{doc['text'][:500]}...\n\n"

        return context

    def chat(self, user_query: str, conversation_history: Optional[list] = None) -> str:
        """
        Generate a response using RAG (Retrieval-Augmented Generation).

        Args:
            user_query: User's question/message
            conversation_history: Previous messages in the conversation (optional)

        Returns:
            AI-generated response based on book knowledge
        """
        try:
            # Step 1: Embed the query
            print(f"[RAG] Embedding query: {user_query[:100]}...")
            query_embedding = self._embed_query(user_query)

            # Step 2: Retrieve relevant context
            print(f"[RAG] Retrieving context from Qdrant...")
            relevant_docs = self._retrieve_context(query_embedding)
            formatted_context = self._format_context(relevant_docs)

            # Step 3: Build the prompt for Gemini
            system_prompt = """You are an expert AI assistant specialized in Physical AI and Humanoid Robotics. 
Your role is to answer questions about the Physical AI and Humanoid Robotics textbook with accuracy and clarity.

Guidelines:
- Use the retrieved context from the book to answer questions accurately
- If the answer is not in the provided context, say "I don't have this information in the current knowledge base"
- Provide clear, educational explanations suitable for students
- Include relevant examples or references when possible
- Be helpful and encouraging to learners"""

            # Build messages for conversation
            messages = []

            # Add conversation history if provided
            if conversation_history:
                messages.extend(conversation_history)

            # Add current query with context
            user_message_with_context = f"""Context from the Physical AI & Robotics Book:

{formatted_context}

---

User Question: {user_query}

Please provide a helpful response based on the above context."""

            messages.append({"role": "user", "content": user_message_with_context})

            # Step 4: Generate response with Gemini
            print(f"[RAG] Generating response with Gemini 2.0 Flash...")
            response = self.genai_model.generate_content(
                messages,
                safety_settings=[
                    genai.types.SafetySetting(
                        category=genai.types.HarmCategory.HARM_CATEGORY_HARASSMENT,
                        threshold=genai.types.HarmBlockThreshold.BLOCK_NONE,
                    ),
                    genai.types.SafetySetting(
                        category=genai.types.HarmCategory.HARM_CATEGORY_HATE_SPEECH,
                        threshold=genai.types.HarmBlockThreshold.BLOCK_NONE,
                    ),
                ],
            )

            return response.text

        except Exception as e:
            error_msg = f"[RAG] Error generating response: {str(e)}"
            print(error_msg)
            return f"I encountered an error while processing your question. Please try again. Error: {str(e)}"

    def get_system_info(self) -> dict:
        """Get information about the chatbot and knowledge base."""
        try:
            collection_info = self.qdrant.get_collection(self.collection_name)
            return {
                "status": "ready",
                "model": self.model_name,
                "collection": self.collection_name,
                "documents_count": collection_info.points_count,
                "vector_size": collection_info.config.params.vectors.size,
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}


# Initialize global chatbot instance
def get_chatbot() -> RAGChatbot:
    """Get or create the RAG chatbot instance."""
    return RAGChatbot(
        collection_name=COLLECTION_NAME,
        top_k=5,
        model_name="gemini-2.0-flash",
    )


if __name__ == "__main__":
    # Test the chatbot
    print("🤖 Physical AI & Robotics RAG Chatbot")
    print("=" * 50)

    chatbot = get_chatbot()

    # Check system info
    info = chatbot.get_system_info()
    print(f"\n📊 Knowledge Base Info:")
    print(f"  Status: {info.get('status')}")
    print(f"  Model: {info.get('model')}")
    print(f"  Collection: {info.get('collection')}")
    print(f"  Documents: {info.get('documents_count', 'N/A')}")

    # Test query
    print(f"\n💬 Testing with a sample question...")
    test_query = "What is Physical AI?"
    response = chatbot.chat(test_query)

    print(f"\n❓ Query: {test_query}")
    print(f"\n🤖 Response:\n{response}")
