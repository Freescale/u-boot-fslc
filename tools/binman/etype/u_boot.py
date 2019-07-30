# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) 2016 Google, Inc
# Written by Simon Glass <sjg@chromium.org>
#
# Entry-type module for U-Boot binary
#

from entry import Entry
from blob import Entry_blob

class Entry_u_boot(Entry_blob):
    def __init__(self, image, etype, node):
        Entry_blob.__init__(self, image, etype, node)

    def GetDefaultFilename(self):
        return 'u-boot.bin'
