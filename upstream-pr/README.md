# Upstream PR payload (esphome/esphome, target branch `dev`)

Copy `esphome/components/si4703/` and `tests/components/si4703/` into a fork of
[esphome/esphome](https://github.com/esphome/esphome) at the same paths, then run
`script/build_codeowners.py`, which adds this line to `CODEOWNERS`:

```
esphome/components/si4703/* @lilmansplace
```

Differences from `components/si4703/` (the copy the devices build from):

- `ESPHOME_LOG_TAG` only (no fallback for released ESPHome versions).
- Tests use the shared I²C bus packages from `tests/test_build_components/common/i2c/`
  with per-platform pin substitutions (esp32-idf, esp8266-ard, rp2040-ard).

Verified on 2026-10-21 against ESPHome `2026.11.0-dev` (commit c74a9da):

- `script/test_build_components.py -e config -c si4703 --isolate si4703`: 3/3 platforms pass.
- `script/test_build_components.py -e compile -c si4703 -t esp8266-ard --isolate si4703`: pass.
- esp32-idf / rp2040-ard: generated code compiles against `dev` headers (host syntax check) and clang-tidy is
  clean; full firmware compile not run (PlatformIO registry blocked in the build sandbox), so CI covers it.
- clang-format, ruff, pylint, `script/ci-custom.py`: clean.

The docs page is in `docs/si4703/` (esphome-docs, target branch `next`).
