# Live Validate

`tools/um982_live_validate.py` est l'outil de validation live autonome du driver `mowgli_unicore_gnss`.

Il permet de tester un recepteur UM980/UM981/UM982 branche en USB/serie sans dependre de MowgliNext ni de ROS 2. Il sait:
- capturer le flux brut serie
- reconnaitre `NMEA`, `Unicore ASCII` et `Unicore binary`
- verifier checksums et CRC
- mesurer les frequences reelles des logs
- comparer `ascii` et `binary` en mode `hybrid`
- remettre le recepteur dans un etat connu avant capture

## Usage

```bash
python3 tools/um982_live_validate.py --help
```

Arguments principaux:
- `--port /dev/ttyUSB0`
- `--baud 921600`
- `--com-port COM1`
- `--duration 30`
- `--profile normal|debug|survey|high_precision`
- `--format ascii|hybrid|binary`
- `--factory-reset`
- `--reset`
- `--save-config`
- `--apply-profile-config`
- `--apply-profile-logs`
- `--discover-log-syntax`
- `--enable-raw`
- `--raw-output capture.bin`
- `--text-output capture.log`
- `--summary summary.json`

## Compatibilite De Syntaxe LOG

Certains firmwares UM98x acceptent bien `LOG <msg> ONTIME <period>` pour
des logs comme `GPGGA` ou `PVTSLNA`, mais utilisent une syntaxe N4
specifique pour d'autres messages:
- `BESTNAVA/BESTNAVB <period>`
- `RTKSTATUSA/RTKSTATUSB <period>`
- `RTCMSTATUSA/RTCMSTATUSB ONCHANGED`
- `GPHPR <period>` avec une sortie qui peut etre `$GNHPR`
- `GPHPR2 ONCHANGED` avec une sortie qui peut etre `$GNHPR2`

Le validateur embarque une table de syntaxe par message:
- `nmea_log_ontime`
- `unicore_direct_period`
- `unicore_onchanged`
- `special_output_name`

Puis il essaye automatiquement des variantes de secours tant qu'il voit
des reponses du type `PARSING FAILED` ou `GRAMMAR ERROR`.

La premiere syntaxe qui n'est plus rejetee est memorisee dans le resume:
- `accepted_log_commands`
- `rejected_log_commands`
- `log_command_syntax_by_message`

Pour tester rapidement uniquement la grammaire des logs sans lancer une
capture longue:

```bash
python3 tools/um982_live_validate.py \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --profile debug \
  --format ascii \
  --apply-profile-logs \
  --discover-log-syntax \
  --duration 0 \
  --summary /tmp/um982-log-syntax.json
```

## Profils

| Profil | But | Format recommande | Logs principaux |
| --- | --- | --- | --- |
| `normal` | validation legere / Nav2 | `ascii` | `GPGGA`, `PVTSLNA/B`, `BESTNAVA/B`, `GPHPR`, `RTKSTATUSA/B`, `RTCMSTATUSA/B` |
| `debug` | diagnostics terrain complets | `ascii` ou `hybrid` | `normal` + `BESTSATA/B`, `SATSINFOA/B`, `AGCA/B`, `HWSTATUSA/B`, `JAMSTATUSA/B`, `FREQJAMSTATUSA/B` |
| `survey` | analyse GNSS avancee | `hybrid` | `debug` avec frequences plus lentes, `OBSVMCMPA/B` optionnel |
| `high_precision` | essais avances / tuning | `hybrid` ou `binary` | `debug` + `CONFIG PVTALG MULTI`, `RTCMDECAUTO`, `RTCMPHASERATE`, `RTCMCLOCKOFFSET` |

## Sequence Recommandee

Premiere remise a plat:

```bash
python3 tools/um982_live_validate.py \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --profile normal \
  --format ascii \
  --factory-reset \
  --apply-profile-config \
  --apply-profile-logs \
  --send-version \
  --summary /tmp/um982-first-pass.json
```

Tests suivants:

```bash
python3 tools/um982_live_validate.py \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --profile normal \
  --format ascii \
  --reset \
  --apply-profile-logs \
  --send-version \
  --summary /tmp/um982-repeat.json
```

Validation debug ASCII:

```bash
python3 tools/um982_live_validate.py \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --duration 30 \
  --profile debug \
  --format ascii \
  --apply-profile-logs \
  --unlog-first \
  --summary /tmp/um982-debug.json
```

Validation survey hybride:

```bash
python3 tools/um982_live_validate.py \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --duration 60 \
  --profile survey \
  --format hybrid \
  --enable-raw \
  --apply-profile-config \
  --apply-profile-logs \
  --raw-output /tmp/um982-survey.bin \
  --text-output /tmp/um982-survey.log \
  --summary /tmp/um982-survey.json
```

## Reset Et Configuration

`FRESET` n'est jamais lance par defaut. `SAVECONFIG` n'est jamais envoye par defaut non plus.

Quand `--factory-reset` est demande, l'outil:
- affiche un warning clair
- envoie `FRESET`
- attend le reboot
- reprobe `115200`, `460800`, `921600`
- remet `CONFIG <com-port> <baud cible>` si necessaire
- rouvre au baud cible

Quand `--reset` est demande, l'outil:
- envoie `RESET`
- attend le reboot
- tente d'abord de rouvrir au meme baud
- retombe sur une reprobe de secours si necessaire

Quand `--apply-profile-config` est demande, l'outil applique une base rover UM98x integree:
- `MODE ROVER SURVEY MOW`
- timeouts RTK/DGPS
- `UNDULATION AUTO`
- constellations GPS/GLO/GAL/BDS actives
- `SBAS` et `AGNSS` desactives
- `SIGNALGROUP` selon le modele si connu

## Resume Produit

Le resume console et JSON contient:
- le baud demande, detecte et reel de capture
- les reponses commandes `ok|unsupported|no_response`
- les syntaxes LOG acceptees/rejetees par message
- les compteurs NMEA / ASCII / binary
- les logs attendus et leur frequence observee
- la presence du fix
- `visible/used`, `CN0`, `diff_age`
- `RTCM alive/stale`
- les deltas `ascii` vs `binary` en mode `hybrid`
- les IDs binaires inconnus et logs ASCII inconnus

## Exemple De Sortie

```text
UM982 Live Validation
Port: /dev/ttyUSB0 requested=921600 baud detected=921600 capture=921600
Profile: survey  Format: hybrid  Duration: 60.0 s
Capture: 1438291 B, 1842 ASCII lines, 1760 binary frames
Checks: NMEA bad=0  ASCII CRC bad=0  binary CRC bad=0  resync=0
Fix: rtk-fixed  visible=31  used=19  diff_age=0.300 s
CN0: ascii mean/max=39.700/47.000  binary mean/max=39.600/47.000
RTCM: alive  age=0.400 s

Command responses
  @921600 VERSION                  ok | #VERSIONA,"UM982","R4.10Build15434"
  @921600 UNLOGALL                unsupported | unsupported command
  @921600 UNLOG                    ok

Conclusion: PASS
```

## ROS 2 Optionnel

Le validateur fonctionne sans ROS 2, mais peut etre croise avec:

```bash
ros2 launch mowgli_unicore_gnss um982_launch.py
ros2 topic hz /gps/fix
ros2 topic echo /gps/diagnostics --once
```

## Integration MowgliNext

MowgliNext ne doit garder qu'un wrapper leger d'integration robot, par exemple `sensors/unicore/validate_live.sh`, qui appelle cet outil du driver.
