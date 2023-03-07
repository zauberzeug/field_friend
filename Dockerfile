FROM zauberzeug/rosys:latest

COPY ./requirements.txt /app

RUN cd /app && python3 -m pip install -r requirements.txt
RUN cd /app && python3 -m pip install --upgrade fastapi-socketio