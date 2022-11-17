FROM python:3.8.10

WORKDIR /home/user/

COPY requirements.txt ./

RUN pip install --no-cache-dir -r requirements.txt

#COPY rttfcar_commander/ ./rttfcar_commander/

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
	bash-completion \
	ffmpeg libsm6 libxext6 
    
CMD ["python3", "/home/user/rttfcar_commander/commander_webserver_with_cam.py"]