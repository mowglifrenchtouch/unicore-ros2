# UM982Driver

Backend ROS 2 avance pour recepteurs GNSS Unicore N4 de la famille UM980, UM981 et UM982.

Le projet ne se limite pas a un simple parser NMEA: il fournit une chaine complete de transport serie, parsing Unicore ASCII et binaire, validation hybride, publication ROS 2, injection RTCM, diagnostics riches et outillage de validation terrain.

## Presentation

`UM982Driver` est implemente dans le package ROS 2 `mowgli_unicore_gnss` et vise les usages GNSS exigeants:

| Domaine | Capacites |
| --- | --- |
| Recepteurs cibles | UM980, UM981, UM982 |
| Formats | ASCII, binary, hybrid |
| Navigation | `GGA`, `PVTSLNA`, `PVTSLNB`, `BESTNAVA`, `BESTNAVB`, heading |
| RTK / corrections | `RTKSTATUSA/B`, `RTCMSTATUSA/B`, injection RTCM ROS 2 |
| Satellites | `BESTSATA/B`, `SATSINFOA/B`, comptage visible/utilise, C/N0 |
| RF / hardware | `AGCA/B`, `HWSTATUSA/B`, etats d'alimentation et AGC |
| Interferences | `JAMSTATUSA/B`, `FREQJAMSTATUSA/B` |
| Observations brutes | `OBSVMCMPB` |
| Runtime | profils `normal`, `debug`, `survey`, `high_precision` |
| Supervision | diagnostics ROS 2 exploitables dans Foxglove et chaines Nav2/outillage ops |

## Fonctionnalites principales

| Fonctionnalite | Description |
| --- | --- |
| Parsing Unicore ASCII | Decode `PVTSLNA`, `BESTNAVA`, `BESTSATA`, `SATSINFOA`, `RTKSTATUSA`, `RTCMSTATUSA`, `AGCA`, `HWSTATUSA`, `JAMSTATUSA`, `FREQJAMSTATUSA` ainsi que `GGA`, `HDT`, `HPR`. |
| Parsing Unicore binary | Decode `BESTNAVB`, `PVTSLNB`, `BESTSATB`, `SATSINFOB`, `RTCMSTATUSB`, `RTKSTATUSB`, `AGCB`, `HWSTATUSB`, `JAMSTATUSB`, `FREQJAMSTATUSB`, `OBSVMCMPB`, `UNIHEADINGB`. |
| Hybrid validation | Compare flux ASCII et binaire pour verifier coherence de fix, altitude, satellites et C/N0. |
| BESTNAV / PVTSLN | Combine navigation detaillee, covariance, qualite de fix, heading et vitesse. |
| BESTSAT / SATSINFO | Expose vue satellite haut niveau et vue detaillee par constellation / bande. |
| RTK / RTCM | Suit etats de correction, age des donnees, flux RTCM recus et etat du moteur RTK. |
| RF / hardware / jamming | Expose AGC, alimentation interne, derive horloge, brouillage large bande et par famille de frequences. |
| Raw observations | Support `OBSVMCMPB` pour validation terrain et futur pipeline d'observations brutes. |
| Live validator | Outil de capture serie hors ROS 2 pour qualification terrain, comparaison hybrid et capture brute. |
| Runtime profiles | Ajuste frequences de logs et profondeur de supervision selon le scenario. |
| Survey / high precision | Oriente le recepteur et la validation vers des modes d'observation et de precision renforces. |

## Architecture

Le backend est structure en briques separees pour garder un pipeline lisible et testable:

| Bloc | Role |
| --- | --- |
| Transport | `src/unicore_transport.cpp` gere le flux serie, la detection des trames ASCII/binary, la resynchronisation et le CRC binaire. |
| Parser ASCII | `src/um982_parser.cpp` decode les phrases NMEA et les logs Unicore ASCII de navigation, RTK, satellites, RF et hardware. |
| Parser binaire | `src/unicore_binary_nav.cpp` decode les messages N4 binaires equivalentes, y compris `OBSVMCMPB`. |
| Diagnostics | `src/um982_node.cpp` construit des `diagnostic_msgs/DiagnosticArray` pour fix, RTK, RTCM, satellites, RF, hardware, jamming, raw observations et sante parser/transport. |
| Integration ROS 2 | Le noeud publie `sensor_msgs/msg/NavSatFix`, `compass_msgs/msg/Azimuth`, consomme `rtcm_msgs/msg/Message` et expose une configuration YAML/launch standard. |
| Validation live | `tools/um982_live_validate.py` permet de valider un flux serie en direct sans lancer ROS 2. |

## Runtime profiles

Les profils pilotent le niveau de verbosite et la profondeur des diagnostics attendus:

| Profil | Usage | Intention |
| --- | --- | --- |
| `normal` | Exploitation courante | Navigation et diagnostics essentiels avec charge limitee. |
| `debug` | Mise au point / integration | Active une supervision satellite, RF, hardware et jamming plus large. |
| `survey` | Leves / qualification terrain | Cadence plus lente pour stabilite, supervision riche et support raw observations. |
| `high_precision` | Tests RTK / precision elevee | Cadence plus rapide sur la navigation et supervision etendue. |

## Output formats

| Format | Description | Cas d'usage |
| --- | --- | --- |
| `ascii` | Backend base sur logs Unicore ASCII et NMEA. | Compatibilite maximale, debogage simple. |
| `binary` | Backend pilote a partir des messages binaires N4. | Precision et richesse des messages binaires. |
| `hybrid` | ASCII + binaire avec comparaison croisee. | Validation de firmware, verification terrain, migration progressive. |

Les bascules principales se font via `config/um982.yaml`:

```yaml
um982_node:
  ros__parameters:
    enable_unicore_binary: true
    use_binary_nav: true
    binary_compare_ascii: true
    use_binary_rtk_diag: true
    use_binary_satellite_diag: true
    use_binary_rtcm_diag: true
    use_binary_rf_diag: true
    use_binary_hw_diag: true
    use_binary_jamming_diag: true
    use_binary_raw_observations: true
```

## Exemple rapide

### Build

```bash
colcon build --packages-select mowgli_unicore_gnss
source install/setup.bash
```

### Launch

```bash
ros2 launch mowgli_unicore_gnss um982_launch.py
```

### Configuration YAML

Exemple minimal pour un backend hybride oriente precision:

```yaml
um982_node:
  ros__parameters:
    port: /dev/gps
    baudrate: 921600
    frame_id: gps
    fix_topic: /gps/fix
    heading_topic: /gps/azimuth
    diagnostics_topic: /gps/diagnostics
    rtcm_topic: /ntrip_client/rtcm

    enable_unicore_binary: true
    use_binary_nav: true
    binary_compare_ascii: true

    enable_rtk_status: true
    enable_rtcm_status: true
    enable_satellite_status: true
    enable_satsinfo: true
    enable_rf_status: true
    enable_hw_status: true
    enable_jamming_status: true

    use_binary_rtk_diag: true
    use_binary_satellite_diag: true
    use_binary_rtcm_diag: true
    use_binary_rf_diag: true
    use_binary_hw_diag: true
    use_binary_jamming_diag: true

    enable_raw_observation_diag: true
    use_binary_raw_observations: true
```

### Live validate

Validation rapide d'un flux hybride sur le port serie:

```bash
python3 tools/um982_live_validate.py \
  --port /dev/ttyUSB0 \
  --baud 921600 \
  --duration 30 \
  --profile survey \
  --format hybrid \
  --enable-raw
```

Le validateur verifie notamment:

| Controle | Ce qui est valide |
| --- | --- |
| Presence des logs | Trames ASCII attendues, trames binaires attendues, IDs inconnus |
| Navigation | Fix, type de solution, deltas position/altitude, heading |
| Satellites | Satellites visibles/utilises, C/N0 moyen, coherence ASCII/binary |
| RTK / RTCM | Age des corrections, activite du moteur RTK, sante du flux RTCM |
| Raw | Presence et volume de `OBSVMCMPB` quand le mode le demande |

## Diagnostics exposes

Le noeud publie un `diagnostic_msgs/msg/DiagnosticArray` sur `diagnostics_topic` avec des statuts pensés pour la supervision operationnelle.

| Diagnostic | Contenu |
| --- | --- |
| Fix | Etat du fix, source active, covariance, age, qualite 3D / DGPS / RTK float / RTK fixed |
| RTK | Etat de solution, carrier solution, age et fraicheur des donnees RTK |
| RTCM | Message RTCM recents, age de correction, activite du lien de corrections |
| Satellites | Nombre visible / utilise, repartition constellation, C/N0, coherence binary/ASCII |
| RF | Niveaux AGC, indices de degradation RF, etat d'alimentation front-end |
| Hardware | Tensions, derive horloge, indicateurs `HWSTATUSA/B` |
| Jamming | Etats `JAMSTATUS` et `FREQJAMSTATUS`, flags par bande, ratios d'interference |
| Raw observations | Disponibilite `OBSVMCMPB`, age des observations, volume d'echantillons |
| Parser / transport | CRC errors, resync, message IDs binaires inconnus, sante du pipeline d'entree |

## Validation terrain

| Axe | Recommandation |
| --- | --- |
| Live validator | Utiliser `tools/um982_live_validate.py` avant une session longue pour verifier profil, format et densite de logs. |
| Hybrid comparison | Preferer `--format hybrid` lors des validations firmware ou des regressions backend pour croiser ASCII et binary. |
| Raw capture | Activer `--enable-raw` en `survey` ou `high_precision` pour confirmer la presence de `OBSVMCMPB`. |
| Survey | En terrain, privilegier `survey` pour une telemetrie riche et une observation stabilisee. |
| High precision | Utiliser `high_precision` pour pousser la cadence nav et suivre les deltas de precision au plus fin. |

## Roadmap

| Sujet | Objectif |
| --- | --- |
| Export RINEX | Sortie standardisee des observations pour traitements externes |
| Post-processing | Pipeline de relecture et d'analyse offline |
| Multipath analysis | Indicateurs et analyses dedies aux environnements difficiles |
| Topic raw observations | Publication ROS 2 dediee des observations brutes |
| Capture replay | Rejeu de sessions terrain pour regression et benchmarking |

## Implementation et provenance

Cette implementation est independante.

Elle n'est pas basee sur une copie ni sur une derivation d'un driver proprietaire.

Le backend est construit a partir de documentation publique Unicore N4, des formats NMEA, des messages RTCM et d'echantillons de trames observes en integration/tests.

## Tests

Le depot contient deja des tests cibles sur les briques critiques:

| Zone | Couverture actuelle |
| --- | --- |
| Transport | extraction de trames binaires, CRC, resynchronisation |
| Parser ASCII | `PVTSLNA`, `BESTNAVA`, `BESTSATA`, `SATSINFOA`, `RTKSTATUSA`, `RTCMSTATUSA`, `AGCA`, `HWSTATUSA`, `JAMSTATUSA`, `FREQJAMSTATUSA` |
| Parser binaire | `BESTNAVB`, `PVTSLNB`, `BESTSATB`, `SATSINFOB`, `RTCMSTATUSB`, `RTKSTATUSB`, `AGCB`, `HWSTATUSB`, `JAMSTATUSB`, `FREQJAMSTATUSB`, `OBSVMCMPB` |
| Outils | smoke tests sur `tools/um982_live_validate.py` |

Pour lancer la suite locale:

```bash
colcon test --packages-select mowgli_unicore_gnss
colcon test-result --verbose
```
