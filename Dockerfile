FROM ubuntu:25.04

LABEL maintainer="alexandre.abadie@inria.fr"

# Install tools required for the build
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        git \
        git-lfs \
        libfontconfig1 \
        libfreetype6 \
        libuuid1 \
        libxcb1 \
        libxext6 \
        libxrender1 \
        wget \
        && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN git lfs install
RUN mkdir -p /opt && git clone https://gitlab.inria.fr/aio/ses-archive.git /opt/ses-archive

ARG SEGGER_STUDIO_VERSION=v824
ARG SEGGER_STUDIO_ARCHIVE=Setup_EmbeddedStudio_${SEGGER_STUDIO_VERSION}_Linux_x86_64.tar.gz

# The SES archive is fetched from a Git repository at https://gitlab.inria.fr/aio/ses-archive
# because it can no longer be downloaded automatically (using wget) from segger.com.
RUN echo 'Installing Segger Studio' >&2 && \
    tar -C /opt -zxf /opt/ses-archive/${SEGGER_STUDIO_ARCHIVE} && \
    /opt/segger_embedded_studio_${SEGGER_STUDIO_VERSION}_linux_x64/install_segger_embedded_studio --accept-license --copy-files-to /opt/segger && \
    rm -rf /opt/arm_segger_embedded_studio_${SEGGER_STUDIO_VERSION}_linux_x64 && \
    rm -rf /opt/ses-archive

RUN mkdir -p /opt/segger/packages

# The packages are copied from a local SES installation on the build host.
# On the build host, run the following command before building the Docker image:
# cp -R ~/.segger/SEGGER\ Embedded\ Studio/v8/packages .
# assuming that the required packages are already installed for SES on the build
# host.
COPY packages/ /opt/segger/packages/
RUN /opt/segger/bin/pkg update -packagesdir /opt/segger/packages
RUN /opt/segger/bin/pkg upgrade -packagesdir /opt/segger/packages
RUN /opt/segger/bin/pkg install -yes -packagesdir /opt/segger/packages CMSIS-CORE_V5 CMSIS-DSP_V5 nRF
RUN chmod +r -R /opt/segger

RUN mkdir /dotbot
RUN git config --global --add safe.directory /dotbot

WORKDIR /dotbot
