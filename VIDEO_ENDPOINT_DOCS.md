# Video Endpoint Documentation

## Overview
The FastAPI backend provides secure video serving endpoints for robot simulation videos. Users can only access videos for robot types they have completed bookings for.

## Endpoints

### `GET /videos/{robot_type}`
Serves video files for completed bookings only.

**Parameters:**
- `robot_type` (string): Robot type (turtlebot, arm, hand)

**Headers:**
- `Authorization: Bearer <JWT_TOKEN>` (required)

**Response:**
- `200 OK`: Video file download with correct content-type `video/mp4`
- `401 Unauthorized`: Invalid or missing JWT token
- `403 Forbidden`: User doesn't have completed booking for this robot type
- `404 Not Found`: Video file doesn't exist or robot type not supported

**Example:**
```bash
curl -H "Authorization: Bearer <JWT_TOKEN>" \
     "http://localhost:8000/videos/turtlebot" \
     --output turtlebot_simulation.mp4
```

### `GET /videos/available`
Returns list of videos available to the current user based on completed bookings.

**Headers:**
- `Authorization: Bearer <JWT_TOKEN>` (required)

**Response:**
```json
{
  "available_videos": ["turtlebot"],
  "completed_bookings": [
    {
      "id": 3,
      "user_id": -1,
      "robot_type": "turtlebot",
      "date": "2024-08-28",
      "start_time": "10:00",
      "end_time": "11:00",
      "status": "completed",
      "created_at": "2025-08-27 18:19:06"
    }
  ]
}
```

## Authentication
All video endpoints require a valid JWT Bearer token. Users must:
1. Login via `/auth/login` to get a JWT token
2. Include the token in the Authorization header: `Authorization: Bearer <token>`

## Authorization
- Users can only access videos for robot types they have **completed** bookings for
- The booking status must be "completed", not "active" or other statuses
- This prevents unauthorized access to simulation content

## CORS Configuration
The endpoints support CORS for frontend access:
- Allowed origins: `http://localhost:3000`, `http://localhost:5173`
- Credentials enabled: `true`
- All methods and headers allowed

## File Structure
Video files are stored at:
```
app/videos/
├── turtlebot_simulation.mp4
├── arm_simulation.mp4
└── hand_simulation.mp4
```

## Error Handling
- Invalid tokens return 401 with proper WWW-Authenticate header
- Missing bookings return 403 with descriptive error message
- Missing files return 404 with administrator contact message
- All errors include JSON response with "detail" field

## Security Features
- JWT token validation on every request
- Booking-based authorization (completed bookings only)
- Secure file serving with proper content-type headers
- No directory traversal vulnerabilities (hardcoded file mapping)

## Testing
Use the comprehensive test script in `/tmp/test_video_endpoint.py` to validate all functionality:
```bash
python3 /tmp/test_video_endpoint.py
```

The test validates:
- Backend health and connectivity
- JWT authentication flow
- Video file existence and permissions
- Endpoint authentication requirements
- CORS configuration
- Access control based on bookings
- Available videos endpoint functionality