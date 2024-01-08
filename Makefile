ros:
	docker compose run --rm --build ros bash

dev:
	docker compose run -u root ros bash
