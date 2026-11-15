SHELL := /bin/bash
.PHONY: deps build sim headless demo test verify preview services
deps:
	bash scripts/install_local_deps.sh
build:
	bash scripts/build.sh
sim:
	bash scripts/run_sim.sh
headless:
	bash scripts/run_sim.sh gui:=false
demo:
	bash -c 'source scripts/env.sh && python3 scripts/demo.py'
test:
	bash -c 'source scripts/env.sh && colcon test --event-handlers console_direct+ && colcon test-result --verbose'
verify:
	bash -c 'source scripts/env.sh && python3 scripts/verify.py'
preview:
	bash -c 'source scripts/env.sh && python3 scripts/render_model.py'
services:
	python3 scripts/service.py
