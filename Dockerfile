FROM quay.io/pypa/manylinux1_x86_64

RUN git clone --progress --verbose https://github.com/eigenteam/eigen-git-mirror.git

RUN cp -r eigen-git-mirror/Eigen/ /usr/include/

RUN curl -L -o boost.tar.bz2 https://sourceforge.net/projects/boost/files/boost/1.61.0/boost_1_61_0.tar.bz2

RUN tar --bzip2 -xf /boost.tar.bz2

RUN cd boost_1_61_0 && CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/opt/_internal/cpython-2.7.15-ucs4/include/python2.7/" ./bootstrap.sh --with-libraries=thread && ./b2 install --prefix=/usr/local/ 

ENV BOOST_ROOT "/boost_1_61_0"

RUN git clone --progress --verbose http://github.com/safijari/pyOpenKarto && ls

RUN /opt/python/cp27-cp27mu/bin/pip install pybind11

RUN cd pyOpenKarto && /opt/python/cp27-cp27mu/bin/python setup.py bdist_wheel

RUN cd dist && auditwheel repair * 

# /opt/python/cp27-cp27mu/bin/pip2 install cmake

# /opt/python/cp27-cp27mu/bin/cmake

# yum install wget and pip pybind11 needed still

# curl -L for redirects

# building boost: needed to specify which python and the .h file location 
# export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/opt/_internal/cpython-2.7.15-ucs4/include/python2.7/"