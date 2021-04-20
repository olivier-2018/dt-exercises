FROM duckietown/dt-commons:daffy-amd64

RUN pip3 install duckietown-world-daffy duckietown-gym-daffy

RUN pip3 uninstall dataclasses

RUN apt udpate && apt install freeglut3-dev

CMD ["bash"]
