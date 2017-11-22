#  @file.
#
#  Copyright 2017 NXP
#
#  This program and the accompanying materials are licensed and made available under
#  the terms and conditions of the BSD License which accompanies this distribution.
#  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#

unset GCC_UTILITY GCC_VERSION MajorVersion MinorVersion

if [ X"$CROSS_COMPILE_64" != X"" ]; then
  ARM64_PREFIX="$CROSS_COMPILE_64"
elif [ X"$CROSS_COMPILE" != X"" ]; then
  ARM64_PREFIX="$CROSS_COMPILE"
else
  ARM64_PREFIX="aarch64-linux-gnu-"
fi

GCC_UTILITY="${ARM64_PREFIX}gcc"
CheckGcc=`which $GCC_UTILITY >/dev/null 2>&1`
if [ "$?" -eq 0 ];then
  GCC_VERSION=`$GCC_UTILITY -v 2>&1 | tail -n 1 | awk '{print $3}'`
  MajorVersion=`echo $GCC_VERSION | cut -d . -f 1`
  MinorVersion=`echo $GCC_VERSION | cut -d . -f 2`
  GCC_ARCH_PREFIX=
  NOTSUPPORTED=0

  case $MajorVersion in
    4)
      case $MinorVersion in
        9)
          GCC_ARCH_PREFIX="GCC49_AARCH64_PREFIX"
        ;;
        *)
          NOTSUPPORTED=1
        ;;
      esac
    ;;
    5)
      case $MinorVersion in
      4)
        GCC_ARCH_PREFIX="GCC5_AARCH64_PREFIX"
      ;;
      *)
        GCC_ARCH_PREFIX="GCC5_AARCH64_PREFIX"
        echo "Warning: ${GCC_UTILITY} version ($MajorVersion.$MinorVersion) has not been tested, please use at own risk."
      ;;
      esac
    ;;
    *)
      NOTSUPPORTED=1
    ;;
  esac

  [ "$NOTSUPPORTED" -eq 1 ] && {
      echo "Error: ${GCC_UTILITY} version ($MajorVersion.$MinorVersion) not supported ."
      unset GCC_UTILITY GCC_VERSION MajorVersion MinorVersion
  }

  [ -n "$GCC_ARCH_PREFIX" ] && {
    export GCC_ARCH_PREFIX="$GCC_ARCH_PREFIX"
    export "$GCC_ARCH_PREFIX=$ARM64_PREFIX"
  }

  unset ARCH
else
    echo "Error: ${GCC_UTILITY} not found. Please check PATH variable."
    unset GCC_UTILITY GCC_VERSION MajorVersion MinorVersion
fi
