import os
import time
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import logging

# Import our modules
from database import DatabaseManager
from auth import auth_manager, get_current_user, require_admin
from services.service_manager import AdminServiceManager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize database
db = DatabaseManager()

# Initialize service manager
service_manager = AdminServiceManager(db)
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Handle application lifespan events"""
    logger.info("🚀 Admin Backend API starting up...")
    logger.info("📊 Database initialized")
    
    # Log service status
    status = service_manager.get_service_status()
    logger.info(f"🔧 Services status: {status['overall_status']}")
    logger.info(f"📋 Core services available: {status['core_services_available']}")
    
    if not status['core_services_available']:
        logger.error("❌ Critical: Core services not available!")
    
    yield
    logger.info("🛑 Admin Backend API shutting down...")

# Create FastAPI app with lifespan
app = FastAPI(title="Robot Admin Backend API", version="1.0.0", lifespan=lifespan)

# Configure CORS with environment variable support
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000,http://localhost:5173").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[origin.strip() for origin in cors_origins],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# API Models

# Authentication Models
class UserRegister(BaseModel):
    name: str
    email: str
    password: str

class UserLogin(BaseModel):
    email: str
    password: str

class TokenResponse(BaseModel):
    access_token: str
    token_type: str
    user: dict

class UserResponse(BaseModel):
    id: int
    name: str
    email: str
    role: str
    created_at: str

# Booking Models
class BookingCreate(BaseModel):
    robot_type: str
    date: str
    start_time: str
    end_time: str

class BookingResponse(BaseModel):
    id: int
    user_id: int
    robot_type: str
    date: str
    start_time: str
    end_time: str
    status: str
    created_at: str

class BookingUpdate(BaseModel):
    status: str

# API Endpoints

@app.get("/")
async def root():
    return {"message": "Robot Programming Console API", "version": "1.0.0"}

# Authentication Endpoints
@app.post("/auth/register", response_model=TokenResponse)
async def register(user_data: UserRegister):
    """Register a new user"""
    auth_service = service_manager.get_auth_service()
    return auth_service.register_user(user_data.name, user_data.email, user_data.password)

@app.post("/auth/login", response_model=TokenResponse)
async def login(user_data: UserLogin):
    """Login user"""
    auth_service = service_manager.get_auth_service()
    return auth_service.login_user(user_data.email, user_data.password)

@app.get("/auth/me", response_model=UserResponse)
async def get_current_user_info(current_user: dict = Depends(get_current_user)):
    """Get current user information"""
    auth_service = service_manager.get_auth_service()
    user = auth_service.get_user_by_token(current_user)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return UserResponse(**user)

# Booking Endpoints
@app.post("/bookings", response_model=BookingResponse)
async def create_booking(booking_data: BookingCreate, current_user: dict = Depends(get_current_user)):
    """Create a new booking"""
    booking_service = service_manager.get_booking_service()
    user_id = int(current_user["sub"])
    booking = booking_service.create_booking(
        user_id=user_id,
        robot_type=booking_data.robot_type,
        date=booking_data.date,
        start_time=booking_data.start_time,
        end_time=booking_data.end_time
    )
    return BookingResponse(**booking)

@app.get("/bookings", response_model=List[BookingResponse])
async def get_user_bookings(current_user: dict = Depends(get_current_user)):
    """Get current user's bookings"""
    booking_service = service_manager.get_booking_service()
    user_id = int(current_user["sub"])
    bookings = booking_service.get_user_bookings(user_id)
    return [BookingResponse(**booking) for booking in bookings]

@app.get("/bookings/all", response_model=List[dict])
async def get_all_bookings(current_user: dict = Depends(require_admin)):
    """Get all bookings (admin only)"""
    booking_service = service_manager.get_booking_service()
    bookings = booking_service.get_all_bookings()
    return bookings

@app.put("/bookings/{booking_id}", response_model=dict)
async def update_booking(booking_id: int, booking_data: BookingUpdate, current_user: dict = Depends(require_admin)):
    """Update booking status (admin only)"""
    booking_service = service_manager.get_booking_service()
    booking = booking_service.update_booking_status(booking_id, booking_data.status)
    return {"message": "Booking updated successfully", "booking": booking}

@app.delete("/bookings/{booking_id}")
async def delete_booking(booking_id: int, current_user: dict = Depends(require_admin)):
    """Delete booking (admin only)"""
    success = db.delete_booking(booking_id)
    if not success:
        raise HTTPException(status_code=404, detail="Booking not found")
    return {"message": "Booking deleted successfully"}

@app.get("/bookings/schedule")
async def get_booking_schedule(start_date: str, end_date: str):
    """Get booking schedule for date range (public)"""
    bookings = db.get_bookings_for_date_range(start_date, end_date)
    return {"bookings": bookings}

# Admin Endpoints
@app.get("/admin/users", response_model=List[UserResponse])
async def get_all_users(current_user: dict = Depends(require_admin)):
    """Get all users (admin only)"""
    users = db.get_all_users()
    return [UserResponse(**user) for user in users]

@app.get("/admin/stats")
async def get_admin_stats(current_user: dict = Depends(require_admin)):
    """Get admin dashboard statistics"""
    users = db.get_all_users()
    bookings = db.get_all_bookings()
    
    total_users = len(users)
    total_bookings = len(bookings)
    active_bookings = len([b for b in bookings if b["status"] == "active"])
    
    return {
        "total_users": total_users,
        "total_bookings": total_bookings,
        "active_bookings": active_bookings,
        "recent_users": users[:5],  # 5 most recent users
        "recent_bookings": bookings[:10]  # 10 most recent bookings
    }

# Service Health Check Endpoints
@app.get("/health")
async def health_check():
    """Get overall system health"""
    status = service_manager.get_service_status()
    return {
        "status": status['overall_status'],
        "timestamp": time.time(),
        "core_services": status['core_services_available'],
        "services": status['services']
    }

@app.get("/health/services")
async def services_status():
    """Get detailed service status"""
    return service_manager.get_service_status()

@app.get("/health/features")
async def available_features():
    """Get available features based on service status"""
    return service_manager.get_available_features()

@app.get("/robots")
def get_available_robots():
    """Get list of available robot types"""
    booking_service = service_manager.get_booking_service()
    return {
        "robots": ["turtlebot", "arm", "hand"],
        "details": booking_service.get_available_robots()
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)