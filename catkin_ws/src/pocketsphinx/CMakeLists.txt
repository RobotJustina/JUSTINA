cmake_minimum_required(VERSION 2.8.3)
project(pocketsphinx)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY vocab
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/recognizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

