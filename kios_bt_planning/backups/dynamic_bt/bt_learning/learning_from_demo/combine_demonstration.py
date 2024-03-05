"""This script provides an util to combine different demonstrations."""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import copy
from distutils.dir_util import copy_tree
import glob
import os
import pathlib
import random
import shutil
import sys
from typing import List

# this is the pick module, not the pick action!!
from pick import pick
import yaml


def includes_path(
    path_list: List[str],
    path: str
) -> bool:
    """Return True if path is the same path as an element of path_list."""
    for p in path_list:
        if os.path.samefile(p, path):
            return True
    return False


arg_parser = argparse.ArgumentParser(
    description='Combine demonstrations from multiple users into a single directory.')
arg_parser.add_argument(
    '--out', '-o', type=pathlib.Path, required=True,
    help='Name of the output directory to create and store the combined demonstrations in.'
)
arg_parser.add_argument(
    '--demos', '-d', nargs='+', required=True,
    help='List of demonstration directories to combine. Supports wildcards.'
)
arg_parser.add_argument(
    '--exclude', '-e', nargs='+',
    help='List of demonstration directories to exclude from --demos. Supports wildcards.\
          To exclude specific demos within a demonstration directory, separate them with "::".\
          Eg. demo::2::4 excludes demonstrations 2 and 4 from demo.'
)
arg_parser.add_argument(
    '--ndemos', '-n', type=int,
    help='Number of demonstrations to include in the destination.\
          They will be selected randomly from the demos specified by --demos and --exclude.\
          If left out, all demonstrations are copied.'
)

args = arg_parser.parse_args()

if args.out.exists():
    answer = input(f'{args.out} already exists. Do you want to replace it? [y/N] ')
    if answer.lower() == 'y':
        shutil.rmtree(args.out)
    else:
        sys.exit(0)

# Build list of directories and resolve wildcards
directories = copy.deepcopy(args.demos)
for i in range(len(directories)-1, -1, -1):
    if '*' in directories[i]:
        resolved = glob.glob(directories[i])
        # Ignore non directories
        for j in range(len(resolved)-1, -1, -1):
            if not os.path.isdir(resolved[j]):
                print(f'{resolved[j]} from wildcard is not a directory. Ignoring...')
                del resolved[j]
        del directories[i]
        directories += resolved
    elif not os.path.isdir(directories[i]):
        print(f'{directories[i]} does not exist or is not a directory.')
        sys.exit(1)

# Build list of exclusions and resolve wildcards
exclusions = []
if args.exclude is not None:
    for exclude in args.exclude:
        if '::' in exclude:
            index = exclude.index('::')
            folder = exclude[:index]
            demos = exclude[index:]

            if '*' in folder:
                exclusions += [f + demos for f in glob.glob(folder)]
            else:
                if os.path.isdir(folder):
                    exclusions.append(exclude)
                else:
                    print(f'Folder {folder} to be excluded does not exist.')
                    sys.exit(1)
        else:
            if '*' in exclude:
                # exclusions += [f for f in glob.glob(exclude)]
                exclusions += list(glob.glob(exclude))
            else:
                if os.path.isdir(exclude):
                    exclusions.append(exclude)
                else:
                    print(f'Folder {exclude} to be excluded does not exist.')
                    sys.exit(1)

# Remove duplicates by converting to set
directories = list(set(directories))

# list of individual demonstrations to exclude from a task
excluded_demos = []

# Go over exclusions
if args.exclude is not None:
    # Check if there are individual demonstrations to exclude
    for exclusion in exclusions:
        if '::' in exclusion:
            parts = exclusion.split('::')
            for n in parts[1:]:
                excluded_demos.append(os.path.join(parts[0], f'demo{n}'))

    excluded = [e for e in exclusions if '::' not in e]

    for i in range(len(directories)-1, -1, -1):
        if includes_path(excluded, directories[i]):
            del directories[i]

# List of frames in the demonstrations
frames = set()
# Detected default frames
default_frames = set()

# Build list of directories to copy
to_copy = []
for folder in directories:
    with open(os.path.join(folder, 'info.yaml'), 'r') as f:
        data = yaml.safe_load(f)

    frames = frames.union(set(data['frames']))
    default_frames.add(data['default_frame'])

    for demo_folder in glob.glob(os.path.join(folder, 'demo*')):
        # Ignore excluded demo folders
        if not includes_path(excluded_demos, demo_folder):
            to_copy.append(demo_folder)

# Sample files to copy
if args.ndemos is not None:
    to_copy = random.sample(to_copy, args.ndemos)

args.out.mkdir()

# Copy files to destination
demo_idx = 1
for folder in to_copy:
    print(f'Copying {folder}...')
    copy_tree(folder, str(args.out / f'demo{demo_idx}'))
    demo_idx += 1

# Write info.yaml
data = {'frames': list(frames)}
if len(default_frames) > 1:
    # Ask the user which one is the default frame
    default_frame, _ = pick(list(default_frames), 'What is the default frame?')
    data['default_frame'] = default_frame
else:
    data['default_frame'] = list(default_frames)[0]

with open(args.out / 'info.yaml', 'w') as f:
    yaml.dump(data, f)
