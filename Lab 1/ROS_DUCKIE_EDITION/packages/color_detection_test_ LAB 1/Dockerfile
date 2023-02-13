FROM duckietown/dt-duckiebot-interface:daffy-arm64v8
# use daffy-arm64v8 if you are using a Duckiebot MOOC Founder's Edition

WORKDIR /color_detector_test

COPY requirements.txt ./

RUN pip install -r requirements.txt

COPY color_detector.py .

CMD python3 ./color_detector.py

