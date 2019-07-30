# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) 2016 Google, Inc
# Written by Simon Glass <sjg@chromium.org>
#
# Entry-type module for x86 VGA ROM binary blob
#

from entry import Entry
from blob import Entry_blob

class Entry_intel_vga(Entry_blob):
    def __init__(self, image, etype, node):
        Entry_blob.__init__(self, image, etype, node)
