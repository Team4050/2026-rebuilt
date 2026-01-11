FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu-24.04

ARG WPI_VERSION
RUN [ -n "$WPI_VERSION" ]

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install --no-install-recommends -y -q \
    sudo curl wget \
    openjdk-17-jdk \
    && apt-get autoremove \
    && apt-get clean \
    && rm -rf /var/lib/{apt,dpkg,cache,log}/

# Dynamically set JAVA_HOME by finding the correct directory
# RUN JAVA_HOME_PATH=$(dirname $(dirname $(readlink -f $(which javac)))) && \
#     ln -s $JAVA_HOME_PATH /usr/lib/jvm/java-17-openjdk && \
#     echo "JAVA_HOME=$JAVA_HOME_PATH" >> /etc/environment

# Set JAVA_HOME for all shells and add to PATH
# ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk
# ENV PATH="$JAVA_HOME/bin:$PATH"

USER vscode

# Install WPILib
RUN ARCH=$(dpkg --print-architecture) && \
    YEAR=$(echo ${WPI_VERSION} | cut -d. -f1) && \
    if [ "$ARCH" = "arm64" ]; then \
        PLATFORM="LinuxArm64"; \
    else \
        PLATFORM="Linux"; \
    fi && \
    wget -nv -O /tmp/wpilib.tar.gz "https://packages.wpilib.workers.dev/installer/v${WPI_VERSION}/${PLATFORM}/WPILib_${PLATFORM}-${WPI_VERSION}.tar.gz" && \
    mkdir -p /home/vscode/wpilib/${YEAR} && \
    tar -xzf /tmp/wpilib.tar.gz -C /home/vscode/wpilib/${YEAR} --strip-components=1 && \
    rm /tmp/wpilib.tar.gz
