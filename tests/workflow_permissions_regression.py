import argparse
import pathlib
import re

import yaml


def require(condition, message):
    if not condition:
        raise SystemExit(message)


parser = argparse.ArgumentParser()
parser.add_argument("--workflow", required=True)
args = parser.parse_args()

workflow = yaml.load(pathlib.Path(args.workflow).read_text(), Loader=yaml.BaseLoader)
permissions = workflow.get("permissions", {})
require(
    isinstance(permissions, dict) and permissions.get("contents") == "read",
    "global contents permission must be read",
)

jobs = workflow.get("jobs", {})
require(isinstance(jobs, dict), "workflow jobs must be a mapping")
require("build" in jobs, "build job missing")
require("tag" in jobs, "separate tag job missing")

build_job = jobs["build"]
build_permissions = build_job.get("permissions", {})
require(
    not build_permissions or build_permissions.get("contents") == "read",
    "build job must not have write permission",
)
build_text = yaml.safe_dump(build_job)
require("createRef" not in build_text, "build job must not create tags")

tag_job = jobs["tag"]
needs = tag_job.get("needs")
if isinstance(needs, list):
    needs_build = needs == ["build"]
else:
    needs_build = needs == "build"
require(needs_build, "tag job must depend only on build")
require(
    tag_job.get("permissions", {}).get("contents") == "write",
    "tag job must have job-local contents write permission",
)

for job_name, job in jobs.items():
    if job_name == "tag":
        continue
    require(
        job.get("permissions", {}).get("contents") != "write",
        f"{job_name} job must not have contents write permission",
    )

condition = re.sub(r"\s+", "", tag_job.get("if", ""))
expected_condition = (
    "${{github.event_name=='push'&&"
    "github.ref==format('refs/heads/{0}',"
    "github.event.repository.default_branch)}}"
)
require(
    condition == expected_condition,
    "tag job must run only for pushes to the repository default branch",
)

tag_text = yaml.safe_dump(tag_job)
require("actions/github-script" in tag_text, "tag job must use GitHub API tagging")
require("createRef" in tag_text, "tag job must create the tag reference")

print("PASS: workflow publication permissions regression")
