FROM python:3.9-slim

ENV PYTHONUNBUFFERED=1

RUN apt update \
    && apt install -y --no-install-recommends python3-tk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY requirements.txt /src/
RUN pip install -r requirements.txt
