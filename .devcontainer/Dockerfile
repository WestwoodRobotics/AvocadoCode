ARG UBUNTU=22.04
ARG TYPE=base

FROM wpilib/ubuntu-${TYPE}:${UBUNTU}

# Create a user named OrangeDynamite
RUN useradd -ms /bin/bash OrangeDynamite

# Add OrangeDynamite to the sudo group
RUN usermod -aG sudo OrangeDynamite

# Set password for the user
RUN echo "OrangeDynamite:2583" | chpasswd

# Install toolchain
RUN curl -SL https://github.com/wpilibsuite/opensdk/releases/download/v2024-1/cortexa9_vfpv3-roborio-academic-2024-x86_64-linux-gnu-Toolchain-12.1.0.tgz | sh -c 'mkdir -p /usr/local && cd /usr/local && tar xzf - --strip-components=2'

USER OrangeDynamite
WORKDIR /home/OrangeDynamite

# Change the shell prompt
RUN echo 'PS1="OrangeDynamite@FRC-2024:\w\$ "' >> ~/.bashrc
















FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

RUN apt update -y
RUN apt install sudo -y

RUN sudo apt install -y curl wget
RUN sudo apt install -y zip unzip


# Install OpenJDK 17
RUN sudo apt-get update && \
    sudo apt-get install -y openjdk-17-jdk && \
    sudo apt-get clean;

# Confirm Java version
RUN java -version

# Define environment variable for Gradle
ENV GRADLE_VERSION=7.6.4 \
    GRADLE_HOME=/opt/gradle \
    PATH=${GRADLE_HOME}/bin:${PATH}

# Install Gradle
RUN wget -q -O gradle.zip "https://services.gradle.org/distributions/gradle-${GRADLE_VERSION}-bin.zip" && \
    sudo unzip gradle.zip -d /opt && \
    sudo ln -s /opt/gradle-${GRADLE_VERSION} /opt/gradle && \
    rm gradle.zip

# Set the JAVA_HOME environment variable
ENV JAVA_HOME /usr/lib/jvm/java-17-openjdk-amd64

# Add the JAVA_HOME bin directory to the PATH
ENV PATH $JAVA_HOME/bin:$PATH


# Create a user named OrangeDynamite
RUN useradd -ms /bin/bash OrangeDynamite

# Add OrangeDynamite to the sudo group
RUN usermod -aG sudo OrangeDynamite

# Set password for the user
RUN echo "OrangeDynamite:2583" | chpasswd

# Install toolchain
RUN curl -SL https://github.com/wpilibsuite/opensdk/releases/download/v2024-1/cortexa9_vfpv3-roborio-academic-2024-x86_64-linux-gnu-Toolchain-12.1.0.tgz | sh -c 'mkdir -p /usr/local && cd /usr/local && tar xzf - --strip-components=2'

USER OrangeDynamite
WORKDIR /home/OrangeDynamite

RUN wget https://github.com/wpilibsuite/allwpilib/archive/refs/tags/v2024.2.1.tar.gz -O wpilib.tar.gz
# Change the shell prompt
RUN sudo echo 'PS1="OrangeDynamite@Apricot_Code:\w\$ "' >> ~/.bashrc
RUN sudo mkdir -p /OrangeDynamite/wpilib/2024
RUN sudo tar -zxvf wpilib.tar.gz -C /OrangeDynamite/wpilib/2024

