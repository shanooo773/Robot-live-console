#!/bin/bash

# Simulation Service Deployment Script
# This script deploys the resource-intensive simulation service

echo "🚀 Deploying Robot Simulation Service..."

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source venv/bin/activate

# Install dependencies
echo "📥 Installing dependencies..."
pip install -r requirements.txt

# Create videos directory
echo "📁 Creating videos directory..."
mkdir -p videos

# Check if Docker is available
if command -v docker &> /dev/null; then
    echo "✅ Docker is available"
    
    # Check if robot-simulation image exists
    if docker image inspect robot-simulation:latest &> /dev/null; then
        echo "✅ robot-simulation Docker image found"
    else
        echo "⚠️  robot-simulation Docker image not found"
        echo "📦 Building Docker image (this may take 15-20 minutes)..."
        cd docker
        docker build -t robot-simulation:latest .
        cd ..
        
        if [ $? -eq 0 ]; then
            echo "✅ Docker image built successfully"
        else
            echo "❌ Failed to build Docker image"
            echo "⚠️  Simulation service will run in fallback mode"
        fi
    fi
else
    echo "⚠️  Docker is not available"
    echo "⚠️  Simulation service will run in fallback mode"
fi

echo "✅ Simulation service setup complete!"
echo "🎯 To start the simulation service:"
echo "   cd simulation-service"
echo "   source venv/bin/activate"
echo "   python main.py"
echo ""
echo "📍 Simulation service will run on: http://localhost:8001"
echo "🔧 Endpoints available:"
echo "   - Code execution: /run-code"
echo "   - Robot info: /robots"
echo "   - Health: /health"
echo "   - Videos: /videos/*"