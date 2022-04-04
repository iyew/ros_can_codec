#!/usr/bin/env python3

import os
import shutil
import cantools

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

# it needs to change dbc file dynamically
DBC_FILE_PATH = os.path.join(SCRIPT_DIR, '..', 'dbc', 'D2S_CAN.dbc')
can_db = cantools.database.load_file(DBC_FILE_PATH)

for msg in can_db.messages:
    file_name = msg.name + '.msg'

    with open(file_name, 'w') as f:
        f.write('#Header header\n\n')

        for sig in msg.signals:
            type_name = ''
            if sig.choices is not None:
                type_name = 'string'
            else:
                type_name = 'float32'
                #if sig.is_float:
                #    type_name = 'float32'
                #else:
                #    if sig.is_signed:
                #        type_name = 'int32'
                #    else:
                #        type_name = 'uint32'
            f.write(type_name + ' ' + sig.name + '\n')

    shutil.move(file_name, '../msg/'+file_name)


# CMakeList.txt find add_message_files and add

