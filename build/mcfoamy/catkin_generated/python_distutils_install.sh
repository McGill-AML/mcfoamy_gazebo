#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/eitan/mcfoamy_gazebo/src/mcfoamy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/eitan/mcfoamy_gazebo/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/eitan/mcfoamy_gazebo/install/lib/python2.7/dist-packages:/home/eitan/mcfoamy_gazebo/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/eitan/mcfoamy_gazebo/build" \
    "/usr/bin/python" \
    "/home/eitan/mcfoamy_gazebo/src/mcfoamy/setup.py" \
    build --build-base "/home/eitan/mcfoamy_gazebo/build/mcfoamy" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/eitan/mcfoamy_gazebo/install" --install-scripts="/home/eitan/mcfoamy_gazebo/install/bin"
