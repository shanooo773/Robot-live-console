"""
Service Manager - Coordinates all application services
Provides health checks and service status information
"""

import logging
from typing import Dict, Any, List
from .docker_service import DockerService
from .auth_service import AuthService
from .booking_service import BookingService
from database import DatabaseManager

logger = logging.getLogger(__name__)

class ServiceManager:
    """
    Manages all application services and provides health check functionality.
    Ensures that core services (auth, booking) remain available even if 
    optional services (docker) fail.
    """
    
    def __init__(self, db: DatabaseManager):
        self.db = db
        self.services = {}
        self._initialize_services()
    
    def _initialize_services(self):
        """Initialize all services"""
        logger.info("🚀 Initializing application services...")
        
        # Initialize core services (must always work)
        try:
            self.auth_service = AuthService(self.db)
            self.services['auth'] = self.auth_service
            logger.info("✅ Auth service initialized")
        except Exception as e:
            logger.error(f"❌ Critical: Auth service failed to initialize: {e}")
            raise
        
        try:
            self.booking_service = BookingService(self.db)
            self.services['booking'] = self.booking_service
            logger.info("✅ Booking service initialized")
        except Exception as e:
            logger.error(f"❌ Critical: Booking service failed to initialize: {e}")
            raise
        
        # Initialize optional services (can fail gracefully)
        try:
            self.docker_service = DockerService()
            self.services['docker'] = self.docker_service
            if self.docker_service.available:
                logger.info("✅ Docker service initialized and available")
            else:
                logger.warning("⚠️ Docker service initialized but unavailable")
        except Exception as e:
            logger.warning(f"⚠️ Docker service failed to initialize: {e}")
            # Create a mock docker service that always returns unavailable
            self.docker_service = None
            self.services['docker'] = None
        
        logger.info("🎉 Service initialization complete")
    
    def get_service_status(self) -> Dict[str, Any]:
        """Get status of all services"""
        status = {
            "overall_status": "operational",
            "core_services_available": True,
            "optional_services": {},
            "services": {}
        }
        
        # Check core services
        core_services = ['auth', 'booking']
        core_available = True
        
        for service_name in core_services:
            service = self.services.get(service_name)
            if service and hasattr(service, 'get_status'):
                service_status = service.get_status()
                status['services'][service_name] = service_status
                if not service_status.get('available', False):
                    core_available = False
            else:
                status['services'][service_name] = {
                    "service": service_name,
                    "available": False,
                    "status": "failed",
                    "error": "Service not initialized"
                }
                core_available = False
        
        status['core_services_available'] = core_available
        
        # Check optional services
        optional_services = ['docker']
        for service_name in optional_services:
            service = self.services.get(service_name)
            if service and hasattr(service, 'get_status'):
                service_status = service.get_status()
                status['services'][service_name] = service_status
                status['optional_services'][service_name] = service_status.get('available', False)
            else:
                status['services'][service_name] = {
                    "service": service_name,
                    "available": False,
                    "status": "unavailable",
                    "error": "Service not initialized or not available"
                }
                status['optional_services'][service_name] = False
        
        # Set overall status
        if not core_available:
            status['overall_status'] = "degraded"
        elif not any(status['optional_services'].values()):
            status['overall_status'] = "limited"
        
        return status
    
    def get_available_features(self) -> Dict[str, List[str]]:
        """Get list of available features based on service status"""
        features = {
            "always_available": [],
            "conditionally_available": [],
            "unavailable": []
        }
        
        # Auth features
        if self.services.get('auth') and self.services['auth'].available:
            features["always_available"].extend([
                "user_registration",
                "user_login", 
                "user_authentication",
                "role_management"
            ])
        else:
            features["unavailable"].extend([
                "user_registration",
                "user_login",
                "user_authentication", 
                "role_management"
            ])
        
        # Booking features
        if self.services.get('booking') and self.services['booking'].available:
            features["always_available"].extend([
                "robot_booking",
                "schedule_management",
                "booking_history",
                "booking_cancellation"
            ])
        else:
            features["unavailable"].extend([
                "robot_booking",
                "schedule_management",
                "booking_history",
                "booking_cancellation"
            ])
        
        # Docker simulation features
        if self.services.get('docker') and self.services['docker'].available:
            features["always_available"].extend([
                "real_robot_simulation",
                "gazebo_simulation",
                "video_recording"
            ])
        else:
            features["conditionally_available"].extend([
                "mock_robot_simulation",
                "simulation_fallback"
            ])
            features["unavailable"].extend([
                "real_robot_simulation", 
                "gazebo_simulation",
                "video_recording"
            ])
        
        return features
    
    def is_core_available(self) -> bool:
        """Check if core services are available"""
        return (self.services.get('auth') and self.services['auth'].available and
                self.services.get('booking') and self.services['booking'].available)
    
    def is_docker_available(self) -> bool:
        """Check if Docker service is available"""
        return (self.services.get('docker') and 
                self.services['docker'] and 
                self.services['docker'].available)
    
    def get_auth_service(self) -> AuthService:
        """Get auth service instance"""
        if not self.services.get('auth'):
            raise RuntimeError("Auth service not available")
        return self.services['auth']
    
    def get_booking_service(self) -> BookingService:
        """Get booking service instance"""
        if not self.services.get('booking'):
            raise RuntimeError("Booking service not available")
        return self.services['booking']
    
    def get_docker_service(self) -> DockerService:
        """Get docker service instance (may be None)"""
        return self.services.get('docker')