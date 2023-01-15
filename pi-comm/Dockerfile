FROM python:3.8.10

WORKDIR /home/user/

ENV TZ=Europe/Brussels
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV CRYPTOGRAPHY_DONT_BUILD_RUST=1

COPY requirements.txt ./

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential gcc libssl-dev \
    bash-completion cargo pkg-config\
    ffmpeg libsm6 libxext6 libffi-dev \
    python3 python3-pip python3-numpy python3-dev python3-opencv

RUN pip install --upgrade pip wheel
RUN pip install --no-cache-dir -r requirements.txt

CMD ["python3", "/home/user/rttfcar_commander/commander_webserver_with_cam.py"]