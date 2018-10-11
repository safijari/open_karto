FROM quay.io/pypa/manylinux1_x86_64

RUN yum install -y boost-devel

RUN git clone --progress --verbose https://github.com/eigenteam/eigen-git-mirror.git

RUN cp -r eigen-git-mirror/Eigen/ /usr/include/

RUN git clone --progress --verbose http://github.com/safijari/pyOpenKarto

RUN cd pyOpenKarto && python2 setup.py build