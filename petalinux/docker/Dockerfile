# PetaLinux 2025.2 build environment
# Usage:
#   1. Download petalinux-v2025.2-*-installer.run from AMD
#   2. Place it in petalinux/docker/ (or set PETALINUX_INSTALLER at build time)
#   3. make -C petalinux docker-build
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# PetaLinux 2025.2 dependencies (UG1144 Table 2)
RUN apt-get update && apt-get install -y --no-install-recommends \
    autoconf automake bc bison build-essential ca-certificates chrpath \
    cpio curl debianutils diffstat expect flex gawk gcc-multilib \
    git gnupg gzip iproute2 iputils-ping kmod libegl1-mesa \
    libglib2.0-dev libncurses5-dev libsdl1.2-dev libssl-dev libtinfo5 \
    libtool lz4 locales lsb-release make net-tools pax python3 \
    python3-distutils python3-gi python3-jinja2 python3-pexpect \
    python3-pip python3-subunit rsync screen socat sudo texinfo \
    tftpd-hpa u-boot-tools unzip wget xterm xz-utils zstd \
    && rm -rf /var/lib/apt/lists/*

# Locale (required by PetaLinux)
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8

# Non-root user (PetaLinux refuses to run as root)
ARG PUSER=plnx
ARG PUID=1000
RUN useradd -m -u ${PUID} -s /bin/bash ${PUSER} \
    && echo "${PUSER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ---------- PetaLinux installer ----------
FROM base AS install

ARG PETALINUX_INSTALLER=petalinux-v2025.2-final-installer.run
COPY ${PETALINUX_INSTALLER} /tmp/installer.run

ARG PETALINUX_DIR=/opt/petalinux/2025.2
RUN chmod +x /tmp/installer.run \
    && mkdir -p ${PETALINUX_DIR} \
    && chown ${PUSER}:${PUSER} ${PETALINUX_DIR}

USER ${PUSER}
RUN /tmp/installer.run --skip_license --dir ${PETALINUX_DIR}

USER root
RUN rm /tmp/installer.run

# ---------- Final image ----------
FROM base AS final

ARG PUSER=plnx
ARG PETALINUX_DIR=/opt/petalinux/2025.2

COPY --from=install ${PETALINUX_DIR} ${PETALINUX_DIR}
RUN chown -R ${PUSER}:${PUSER} ${PETALINUX_DIR}

ENV PETALINUX=${PETALINUX_DIR}

# Workspace mount point
RUN mkdir -p /work && chown ${PUSER}:${PUSER} /work
WORKDIR /work

USER ${PUSER}

COPY --chown=${PUSER}:${PUSER} entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
