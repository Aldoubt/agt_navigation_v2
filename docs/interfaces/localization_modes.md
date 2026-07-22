# Relocalization Modes

`/agt/localization/set_mode` uses `agt_interfaces/srv/SetLocalizationMode`.

- `MANUAL_ONLY`: only `/initialpose` and explicit project Action requests; no
  status-driven automatic request.
- `AUTO_ON_START`: waits for map/cloud/Action prerequisites and sends one
  bounded `MODE_AUTO_SEARCH`; failure becomes LOST and waits for human handling.
- `AUTO_RECOVERY`: only triggers from DEGRADED/RECOVERING with fresh registered
  cloud and map identity, then observes a maximum attempt count, cooldown,
  candidate limit, and total timeout. Failure exhausts recovery and leaves LOST.

The coordinator sends only `/agt/localization/relocalize`. It publishes no TF,
velocity, safety enable, or chassis command. Existing safety localization guards
therefore stop navigation input when localization degrades.

The Web projection exposes every `LocalizationStatus` metric including state,
quality booleans, errors, backend/candidate identity, map hash, fitness/overlap/
inlier/ambiguity, innovations, runtime, candidate counts and consecutive result
counts. Display margins are computed from the current
`agt_localization/config/relocalization.yaml`; no second frontend threshold set
is embedded in JavaScript.
