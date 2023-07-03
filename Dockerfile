FROM zauberzeug/rosys:0.8.5

COPY ./requirements.txt /app

RUN cd /app && python3 -m pip install -r requirements.txt
# RUN cd /app && python3 -m pip install --upgrade fastapi-socketio