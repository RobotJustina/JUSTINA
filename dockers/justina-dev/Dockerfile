from rmartella/justina-nn:latest

WORKDIR $HOME
USER biorobotica

# setup entrypoint
COPY ./ros_workspace_entrypoint.sh /
COPY ./processMonitor.py $HOME

RUN mkdir .config/terminator -p
COPY ./config .config/terminator/

USER root

RUN chown biorobotica.biorobotica .config/terminator/config

USER biorobotica
WORKDIR $HOME
RUN git clone https://github.com/RobotJustina/Mobile_Mask_RCNN
RUN git clone https://github.com/rmartella/pycococreator
RUN cd Mobile_Mask_RCNN && git pull origin master

USER root
WORKDIR $HOME
RUN cd Mobile_Mask_RCNN && pip install -r requirements.txt  && python setup.py install
RUN cd pycococreator && python setup.py install
USER biorobotica

ENTRYPOINT ["/ros_entrypoint.sh","/dlib_entrypoint.sh","/ros_workspace_entrypoint.sh"]
CMD ["bash"]

WORKDIR $HOME
