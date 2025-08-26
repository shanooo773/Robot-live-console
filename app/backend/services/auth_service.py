"""
Authentication Service - Handles user authentication and authorization
This service is completely independent of Docker and should always be available.
"""

import logging
from typing import Optional, Dict, Any
from fastapi import HTTPException
from auth import auth_manager
from database import DatabaseManager

logger = logging.getLogger(__name__)

class AuthServiceException(Exception):
    """Exception raised when authentication service encounters an error"""
    pass

class AuthService:
    """
    Service for handling user authentication and authorization.
    This service operates independently of Docker and other services.
    """
    
    def __init__(self, db: DatabaseManager):
        self.db = db
        self.auth_manager = auth_manager
        self.available = True
        self.status = "available"
        logger.info("✅ Auth service initialized successfully")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current service status"""
        return {
            "service": "auth",
            "available": self.available,
            "status": self.status,
            "features": ["registration", "login", "jwt_tokens", "role_management"]
        }
    
    def register_user(self, name: str, email: str, password: str) -> Dict[str, Any]:
        """Register a new user"""
        try:
            user = self.db.create_user(name, email, password)
            token = self.auth_manager.create_access_token(
                data={"sub": str(user["id"]), "email": user["email"], "role": user["role"]}
            )
            return {
                "access_token": token,
                "token_type": "bearer",
                "user": user
            }
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        except Exception as e:
            logger.error(f"Registration failed: {e}")
            raise AuthServiceException(f"Registration failed: {str(e)}")
    
    def login_user(self, email: str, password: str) -> Dict[str, Any]:
        """Login user and return token"""
        try:
            user = self.db.authenticate_user(email, password)
            if not user:
                raise HTTPException(status_code=401, detail="Invalid email or password")
            
            token = self.auth_manager.create_access_token(
                data={"sub": str(user["id"]), "email": user["email"], "role": user["role"]}
            )
            return {
                "access_token": token,
                "token_type": "bearer",
                "user": user
            }
        except HTTPException:
            raise
        except Exception as e:
            logger.error(f"Login failed: {e}")
            raise AuthServiceException(f"Login failed: {str(e)}")
    
    def get_user_by_token(self, token_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Get user information from token data"""
        try:
            user_id = int(token_data["sub"])
            user = self.db.get_user_by_id(user_id)
            if not user:
                return None
            return user
        except Exception as e:
            logger.error(f"Failed to get user by token: {e}")
            return None
    
    def verify_admin_role(self, user: Dict[str, Any]) -> bool:
        """Verify if user has admin role"""
        return user.get("role") == "admin"