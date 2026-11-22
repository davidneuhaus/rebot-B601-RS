# Humanoid-Logbuch (lokaler Entwurf)

Statische Seite, erzeugt aus Markdown. Keine Abhängigkeiten außer Python 3 (Generator) und ffmpeg (Medien).

## Struktur
- `entries/en/YYYY-MM-DD.md` (primär, Englisch) und `entries/de/YYYY-MM-DD.md` (Deutsch) – ein Eintrag pro Arbeitstag, gleicher Dateiname in beiden Ordnern
- Ausgabe: Englisch unter `/`, Deutsch unter `/de/`, Sprachumschalter EN | DE im Header; UI-Texte stehen in `T` in `tools/build.py`, Projekttexte in `project.json` unter `i18n.en`/`i18n.de` (Meilensteine mit `en`/`de`)
- (EN-Überschriften: What we did, Mindset, Wins, Fails, Learnings, Sources, Code status)
- `project.json` – Titel, Intro, „Stand des Projekts“, Meilensteine, Repo-URL, Cover (`site/media/<cover>`)
- `tools/build.py` – Generator · `tools/media.py` – Medien-Helfer · `tools/assets/` – CSS/JS (Quelle)
- `site/` – fertige Seite (wird überschrieben, Deployment-Ordner) · `site/media/` – Bilder/Videos
- `media-inbox/` – Rohmaterial ablegen · `screens/` – Screenshots (`tools/shot.py`)

## Neuen Eintrag anlegen
Datei `entries/en/2026-11-22.md` und `entries/de/2026-11-22.md` erstellen:

    ---
    title: Kurzer Titel des Tages
    date: 2026-11-22
    hours: 2.5            # optional
    summary: Ein bis zwei Sätze, erscheint in der Kapitelliste.
    commit: abc1234       # optional, verlinkt Commit
    hero: ttag4-hero.webp  # optional Titelbild (aus site/media)
    status: PLATZHALTER   # nur für Dummy-Einträge
    # Zeilen mit "~ Video coming soon: ..." (de: "~ Video folgt: ...") erzeugen einen dezenten Hinweis; "![klein|Text](bild.webp)" zeigt ein Bild schmal
    ---
    Optionaler Einleitungsabsatz.

    ## Was wir gemacht haben
    ## Denkweise
    ## Wins
    ## Fails
    ## Learnings
    ## Quellen
    ## Code-Stand

Nicht benötigte Abschnitte einfach weglassen. Die Tag-Nummer ergibt sich aus der Sortierung nach Datum. Kapitelliste und Zeitleiste auf der Startseite zeigen die neuesten Einträge oben (absteigend); Vor/Zurück auf Eintragsseiten: älterer / neuerer Tag.
Markdown: Absätze, `-`/`1.` Listen, `**fett**`, `*kursiv*`, `` `code` ``, `[Link](url)`, `> Zitat`.

## Medien
    python3 tools/media.py image media-inbox/foto.jpg tag4-aufbau          # -> site/media/tag4-aufbau.webp (max 1600px)
    python3 tools/media.py video media-inbox/clip.mov 00:01:05 8 tag4-clip  # Start, Dauer(s) -> mp4 (<5 MB, max 1280px, ohne Ton) + .jpg Poster
    # --audio behält Ton, --width 960 verkleinert

Das Skript gibt die Markdown-Zeile aus; in den Eintrag einfügen:

    ![Bildunterschrift](tag4-aufbau.webp)
    ![Clip: Teleop-Test](tag4-clip.mp4)

Videos laufen stumm, in Schleife, `playsinline`, und starten erst im Sichtbereich; Poster wird automatisch genutzt.

## Bauen & ansehen
    python3 tools/build.py
    cd site && python3 -m http.server 8091     # http://localhost:8091
    python3 tools/shot.py                       # Screenshots (Playwright + Chromium)

Hinweis: Port 8091 ist auf dieser Box durch einen anderen Prozess belegt, daher 8091.
Vor dem Livegang: `repo_public` in `project.json` auf `true` setzen, sobald der Code veröffentlicht ist (dann erscheint der GitHub-Button).
