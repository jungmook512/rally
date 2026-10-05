"""RALLY 대시보드 — FastAPI + WebSocket(Redis pub/sub)
담당: 정동휘
"""
from fastapi import FastAPI
app = FastAPI(title="RALLY dashboard")

@app.get("/health")
def health():
    return {"ok": True}
