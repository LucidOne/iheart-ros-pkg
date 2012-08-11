#! /usr/bin/env python

import roslib

roslib.load_manifest('urdf_compose')
import compose

if __name__ == "__main__":
    compose.main()
