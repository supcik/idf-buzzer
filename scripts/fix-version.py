from git import Repo
import semver
import re
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

repo = Repo()
assert not repo.bare

head = repo.head
tags = [t for t in repo.tags if t.commit == head.commit]
if len(tags) == 0:
    print("No tag found for the current commit.")
    exit(1)

version = tags[0].name
version = re.sub(r"^[vV]", "", version)  # Remove leading 'v' if present

semver_version = semver.VersionInfo.parse(version)

with open("buzzer/idf_component.yml", "r") as f:
    data = load(f, Loader=Loader)
    data["version"] = str(semver_version)

with open("buzzer/idf_component.yml", "w") as f:
    dump(data, f, Dumper=Dumper, default_flow_style=False, allow_unicode=True, sort_keys=False)

