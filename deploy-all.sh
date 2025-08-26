#!/bin/bash

# Robot Live Console - Complete Deployment Script
# Deploys all separated services: admin backend, simulation service, and frontend

echo "🚀 Deploying Robot Live Console - Separated Architecture"
echo "================================================================="

# Deploy admin backend
echo ""
echo "🔧 Step 1: Deploying Admin Backend..."
cd admin-backend
./deploy.sh
cd ..

# Deploy simulation service
echo ""
echo "🔧 Step 2: Deploying Simulation Service..."
cd simulation-service
./deploy.sh
cd ..

# Deploy frontend
echo ""
echo "🔧 Step 3: Deploying Frontend..."
cd frontend
./deploy.sh
cd ..

echo ""
echo "================================================================="
echo "🎉 DEPLOYMENT COMPLETE!"
echo "================================================================="
echo ""
echo "📍 Service URLs:"
echo "   🔐 Admin Backend:      http://localhost:8000"
echo "   🤖 Simulation Service: http://localhost:8001" 
echo "   💻 Frontend:           http://localhost:3000"
echo ""
echo "🚀 To start all services:"
echo ""
echo "   Terminal 1 - Admin Backend:"
echo "   cd admin-backend && source venv/bin/activate && python main.py"
echo ""
echo "   Terminal 2 - Simulation Service:"
echo "   cd simulation-service && source venv/bin/activate && python main.py"
echo ""
echo "   Terminal 3 - Frontend:"
echo "   cd frontend && npm run dev"
echo ""
echo "🎯 Benefits of Separated Architecture:"
echo "   ✅ Lightweight admin backend (deployable on small VPS)"
echo "   ✅ Resource-intensive simulation isolated"
echo "   ✅ Independent scaling and deployment"
echo "   ✅ Same UI/UX experience"
echo "   ✅ All original functionality preserved"