from rmartella/justina-yolo:latest

WORKDIR $HOME
USER biorobotica

# setup entrypoint
COPY ./ros_workspace_entrypoint.sh /

RUN mkdir .config/terminator -p
COPY ./config .config/terminator/

USER root

RUN chown biorobotica.biorobotica .config/terminator/config

USER biorobotica

ENTRYPOINT ["/ros_entrypoint.sh","/dlib_entrypoint.sh","/ros_workspace_entrypoint.sh"]
CMD ["bash"]

WORKDIR $HOME
