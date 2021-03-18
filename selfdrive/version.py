#!/usr/bin/env python3
import os
import subprocess
from typing import List, Optional
from common.basedir import BASEDIR
from selfdrive.swaglog import cloudlog
from common.op_params import opParams
from common.travis_checker import travis

cloak = opParams().get('cloak') if not travis else True

def run_cmd(cmd: List[str]) -> str:
    return subprocess.check_output(cmd, encoding='utf8').strip()


def run_cmd_default(cmd: List[str], default: Optional[str] = None) -> Optional[str]:
  try:
    return run_cmd(cmd)
  except subprocess.CalledProcessError:
    return default


def get_git_commit(branch: str = "HEAD", default: Optional[str] = None) -> Optional[str]:
  if cloak:
    return "a3bef4c46274e16ca6d1b71c8ef5d02f942f7bd0"
  return run_cmd_default(["git", "rev-parse", branch], default=default)


def get_git_branch(default: Optional[str] = None) -> Optional[str]:
  if cloak:
    return "release2"
  return run_cmd_default(["git", "rev-parse", "--abbrev-ref", "HEAD"], default=default)


def get_git_full_branchname(default: Optional[str] = None) -> Optional[str]:
  if cloak:
    return "origin/release2"
  return run_cmd_default(["git", "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"], default=default)


def get_git_remote(default: Optional[str] = None) -> Optional[str]:
  if cloak:
    return "https://github.com/commaai/openpilot.git"
  try:
    local_branch = run_cmd(["git", "name-rev", "--name-only", "HEAD"])
    tracking_remote = run_cmd(["git", "config", "branch." + local_branch + ".remote"])
    return run_cmd(["git", "config", "remote." + tracking_remote + ".url"])
  except subprocess.CalledProcessError:  # Not on a branch, fallback
    return run_cmd_default(["git", "config", "--get", "remote.origin.url"], default=default)


with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "common", "version.h")) as _versionf:
  version = _versionf.read().split('"')[1]

prebuilt = os.path.exists(os.path.join(BASEDIR, 'prebuilt'))

training_version: bytes = b"0.2.0"
terms_version: bytes = b"2"

dirty: bool = True
comma_remote: bool = False
tested_branch: bool = False
origin = get_git_remote()
branch = get_git_full_branchname()
commit = get_git_commit()

if (origin is not None) and (branch is not None):
  try:
    if cloak:
      comma_remote = origin.startswith('git@github.com:commaai') or origin.startswith('https://github.com/commaai')
    else:
      comma_remote = origin.startswith('git@github.com:arne182') or origin.startswith('https://github.com/arne182')
    tested_branch = get_git_branch() in ['release2', 'release3', 'release4', 'release5', 'release6']

    dirty = not comma_remote
    if not cloak:
      dirty = dirty or (subprocess.call(["git", "diff-index", "--quiet", branch, "--"]) != 0)

    if dirty:
      dirty_files = subprocess.check_output(["git", "diff-index", branch, "--"], encoding='utf8')
      commit = subprocess.check_output(["git", "rev-parse", "--verify", "HEAD"], encoding='utf8').rstrip()
      origin_commit = subprocess.check_output(["git", "rev-parse", "--verify", branch], encoding='utf8').rstrip()
      cloudlog.event("dirty comma branch", version=version, dirty=dirty, origin=origin, branch=branch,
                     dirty_files=dirty_files, commit=commit, origin_commit=origin_commit)

  except subprocess.CalledProcessError:
    dirty = True
    cloudlog.exception("git subprocess failed while checking dirty")


if __name__ == "__main__":
  print("Dirty: %s" % dirty)
  print("Version: %s" % version)
  print("Remote: %s" % origin)
  print("Branch: %s" % branch)
  print("Prebuilt: %s" % prebuilt)
