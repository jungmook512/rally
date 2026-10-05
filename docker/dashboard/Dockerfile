FROM python:3.12-slim
WORKDIR /app
COPY dashboard/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY dashboard/ .
EXPOSE 8000
