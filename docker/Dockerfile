# Use Python base image for simplified simulation
FROM python:3.9-slim

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONPATH=/opt/simulation:$PYTHONPATH

# Create simulation directory
WORKDIR /opt/simulation
RUN mkdir -p /opt/simulation/robots /opt/simulation/scripts

# Copy robot descriptions and simulation files
COPY robots/ /opt/simulation/robots/
COPY scripts/ /opt/simulation/scripts/

# Create main simulation runner script
COPY scripts/run_simulation.py /opt/simulation/run_simulation.py

# Create a simplified entrypoint script
RUN echo '#!/bin/bash\n\
export PYTHONPATH=/opt/simulation:$PYTHONPATH\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["python3", "/opt/simulation/run_simulation.py", "--help"]
