#!/usr/bin/env python3
"""Humanoid Logbook generator: entries/{en,de}/*.md -> site/ (EN) and site/de/ (DE). No dependencies."""
import re, html, shutil, datetime, json
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
ENT, SITE, ASSETS = ROOT/"entries", ROOT/"site", ROOT/"tools"/"assets"

CFG = json.loads((ROOT/"project.json").read_text(encoding="utf-8"))
E = html.escape

def inline(t):
    t = E(t, quote=False)
    t = re.sub(r"`([^`]+)`", r"<code>\1</code>", t)
    t = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"(?<![\w*])\*([^*]+)\*(?!\w)", r"<em>\1</em>", t)
    t = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', t)
    return t

def media(alt, src, prefix):
    small = alt.startswith("klein|"); alt = alt[6:] if small else alt
    fcls = " class=\"small\"" if small else ""
    cap = f"<figcaption>{inline(alt)}</figcaption>" if alt else ""
    p = src if src.startswith("http") else prefix + src
    if src.lower().endswith((".mp4", ".webm")):
        poster = re.sub(r"\.\w+$", ".jpg", p)
        has = src.startswith("http") or (SITE/"media"/re.sub(r"\.\w+$", ".jpg", Path(src).name)).exists()
        pa = f' poster="{poster}"' if has else ""
        return f'<figure class="video"><video src="{p}"{pa} muted loop playsinline autoplay controls preload="metadata"></video>{cap}</figure>'
    return f'<figure{fcls}><img src="{p}" alt="{E(alt)}" loading="lazy">{cap}</figure>'

def md(text, prefix):
    out, para, lst, quote = [], [], None, []
    def flush():
        nonlocal para, lst, quote
        if para: out.append("<p>"+inline(" ".join(para))+"</p>"); para=[]
        if lst: out.append(f"<{lst[0]}>"+"".join(f"<li>{inline(i)}</li>" for i in lst[1])+f"</{lst[0]}>"); lst=None
        if quote: out.append("<blockquote>"+inline(" ".join(quote))+"</blockquote>"); quote=[]
    for line in text.split("\n"):
        s = line.strip()
        m = re.fullmatch(r"!\[(.*?)\]\((.+?)\)", s)
        if not s: flush()
        elif m: flush(); out.append(media(m[1], m[2], prefix))
        elif s.startswith("~ "): flush(); out.append('<div class="soon">'+inline(s[2:])+'</div>')
        elif s.startswith("> "): 
            if para or lst: flush()
            quote.append(s[2:])
        elif re.match(r"[-*] ", s):
            if not lst or lst[0]!="ul": flush(); lst=("ul",[])
            lst[1].append(s[2:])
        elif re.match(r"\d+\. ", s):
            if not lst or lst[0]!="ol": flush(); lst=("ol",[])
            lst[1].append(re.sub(r"^\d+\. ","",s))
        else:
            if lst or quote: flush()
            para.append(s)
    flush(); return "\n".join(out)

def parse(path):
    raw = path.read_text(encoding="utf-8")
    m = re.match(r"---\n(.*?)\n---\n(.*)", raw, re.S)
    meta, body = {}, raw
    if m:
        body = m[2]
        for l in m[1].split("\n"):
            if ":" in l:
                k, v = l.split(":",1); meta[k.strip()] = v.strip().strip('"')
    secs, cur, pre = {}, None, []
    for l in body.split("\n"):
        h = re.match(r"##\s+(.*)", l)
        if h: cur = h[1].strip(); secs[cur]=[]
        elif cur: secs[cur].append(l)
        else: pre.append(l)
    return meta, "\n".join(pre), {k:"\n".join(v) for k,v in secs.items()}


ICONS = ["🛠","💭","🏆","💥","📌","🔗","&lt;/&gt;"]
T = {
 "en": dict(
  sections=["What we did","Mindset","Wins","Fails","Learnings","Sources","Code status"],
  months="January February March April May June July August September October November December".split(),
  wd="Monday Tuesday Wednesday Thursday Friday Saturday Sunday".split(),
  fmt=lambda d,T: f"{T['wd'][d.weekday()]}, {T['months'][d.month-1]} {d.day}, {d.year}",
  brand="Humanoid Logbook", nav=["Cover","Chapters","Timeline","About"], day="Day", min="min", read="min read", work="Working time",
  ph="PLACEHOLDER", start="Start with Day 1 →", latest="Latest entry", kicker="A build logbook", stand="Project status",
  progress="Progress", entries="Entries", last="Last entry", timeline="Timeline", chapters="Chapters",
  prev="← Older day", next="Newer day →", endnote="End of the logbook so far", back="Back to overview",
  footer="Humanoid Logbook · a project by {a} · draft, built locally on {d}", about_k="About the project", about_h="Why this logbook?",
  notpub="Not published yet", pubtxt="Code will be published once approved.", gh="Open GitHub repo ↗", fol_h="Follow along", fol_p="The code and this logbook live on GitHub. Star the repo to bookmark it, or hit Watch to get notified about new activity.", fol_star="★ Star on GitHub", fol_watch="Watch for updates", fol_feed="Atom feed of commits", fol_code="Code on GitHub", ghabout="Code on GitHub ↗", page_about="About the project", contact="Contact", ct_p="Questions or ideas? The best place is GitHub:", ct_issues="open an issue", ct_disc="join the Discussions", ct_mail="Prefer email? Write to", lic="MIT License", lic_p="Code is open source under the MIT License.", contact_p="Questions or feedback?"),
 "de": dict(
  sections=["Was wir gemacht haben","Denkweise","Wins","Fails","Learnings","Quellen","Code-Stand"],
  months="Januar Februar März April Mai Juni Juli August September Oktober November Dezember".split(),
  wd="Montag Dienstag Mittwoch Donnerstag Freitag Samstag Sonntag".split(),
  fmt=lambda d,T: f"{T['wd'][d.weekday()]}, {d.day}. {T['months'][d.month-1]} {d.year}",
  brand="Humanoid-Logbuch", nav=["Titelseite","Kapitel","Zeitleiste","Über das Projekt"], day="Tag", min="Min.", read="Min. Lesezeit", work="Arbeitszeit",
  ph="PLATZHALTER", start="Mit Tag 1 beginnen →", latest="Neuester Eintrag", kicker="Ein Bau-Logbuch", stand="Stand des Projekts",
  progress="Fortschritt", entries="Einträge", last="Letzter Eintrag", timeline="Zeitleiste", chapters="Kapitel",
  prev="← Älterer Tag", next="Neuerer Tag →", endnote="Ende des bisherigen Logbuchs", back="Zurück zur Übersicht",
  footer="Humanoid-Logbuch · ein Projekt von {a} · Entwurf, lokal gebaut am {d}", about_k="Über das Projekt", about_h="Warum dieses Logbuch?",
  notpub="Noch nicht veröffentlicht", pubtxt="Code wird veröffentlicht, sobald freigegeben.", gh="GitHub-Repo öffnen ↗", fol_h="Dabei bleiben", fol_p="Code und dieses Logbuch liegen auf GitHub. Gib dem Repo einen Stern als Lesezeichen oder klick auf „Watch“, um über Neues informiert zu werden.", fol_star="★ Stern auf GitHub", fol_watch="Updates verfolgen (Watch)", fol_feed="Atom-Feed der Commits", fol_code="Code auf GitHub", ghabout="Code auf GitHub ↗", page_about="Über das Projekt", contact="Kontakt", ct_p="Fragen oder Ideen? Am besten über GitHub:", ct_issues="ein Issue eröffnen", ct_disc="in den Discussions mitreden", ct_mail="Lieber per E-Mail? Schreib an", lic="MIT-Lizenz", lic_p="Der Code ist Open Source unter der MIT-Lizenz.", contact_p="Fragen oder Feedback?"),
}

def contact_block(t):
    m = CFG.get("contact_email")
    if not m: return ""
    u, d = m.split("@"); r = lambda x: x[::-1]
    u2 = E(r(u)); d2 = E(r(d)); u_s = E(u); d_s = E(d.replace(".", " [dot] "))
    gh = E(CFG["repo"]).rstrip("/")
    disc = f' · <a href="{gh}/discussions">{t["ct_disc"]}</a>' if CFG.get("repo_public") else ""
    return (f'<h2>{t["contact"]}</h2><p>{t["ct_p"]} <a href="{gh}/issues">{t["ct_issues"]}</a>{disc}.</p>'
            f'<p>{t["ct_mail"]} <span class="obf" data-u="{u2}" data-d="{d2}">{u_s} [at] {d_s}</span></p>'
            """<script>document.querySelectorAll('.obf').forEach(function(e){var r=function(x){return x.split('').reverse().join('')};"""
            """var a=r(e.dataset.u)+String.fromCharCode(64)+r(e.dataset.d);var l=document.createElement('a');l.href='mail'+'to:'+a;l.textContent=a;e.replaceWith(l)})</script>""")

def lic_link(t):
    if not CFG.get("repo_public"): return ""
    return f' · <a href="{E(CFG["repo"]).rstrip("/")}/blob/main/LICENSE">{t["lic"]}</a>'

def follow(t):
    if not CFG.get("repo_public"): return ""
    u = E(CFG["repo"]).rstrip("/")
    return (f'<section class="wrap follow"><div class="card"><h2>{t["fol_h"]}</h2><p>{t["fol_p"]} {t["lic_p"]}</p>'
            f'<p class="code-links"><a class="btn" href="{u}">{t["fol_code"]} ↗</a> <a class="btn ghost" href="{u}/stargazers">{t["fol_star"]}</a> '
            f'<a class="btn ghost" href="{u}/subscription">{t["fol_watch"]}</a> <a class="btn ghost" href="{u}/commits/main.atom">{t["fol_feed"]}</a></p>'
            + '</div></section>')

def layout(L, title, body, depth, active, path, desc=""):
    t = T[L]; r = "../"*depth                    # root of this language
    root = r + ("../" if L=="de" else "")        # site root (assets)
    names = ["home","chapters","timeline","about"]; hrefs = ["index.html","index.html#kapitel","index.html#zeitleiste","about.html"]
    nav = "".join(f'<a href="{r}{h}"{" class=on" if k==active else ""}>{n}</a>' for k,h,n in zip(names,hrefs,t["nav"]))
    en_href = (r+path) if L=="en" else (root+path); de_href = (r+"de/"+path) if L=="en" else (r+path)
    sw = f'<span class="lang"><a href="{en_href}"{" class=on" if L=="en" else ""} hreflang="en" lang="en">EN</a><i>|</i><a href="{de_href}"{" class=on" if L=="de" else ""} hreflang="de" lang="de">DE</a></span>'
    foot = t["footer"].format(a=E(CFG["author"]), d=datetime.date.today().strftime("%d.%m.%Y" if L=="de" else "%Y-%m-%d"))
    return f'''<!doctype html><html lang="{L}"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{E(title)}</title><meta name="description" content="{E(desc)}"><link rel="stylesheet" href="{root}assets/style.css"></head>
<body><header class="top"><a class="brand" href="{r}index.html"><span class="dot"></span>{t["brand"]}</a><nav>{nav}{sw}</nav></header>
{body}
{follow(t)}
<footer><p>{foot}{lic_link(t)}</p></footer>
<script src="{root}assets/app.js"></script></body></html>'''

def load(L):
    out = []
    for p in sorted((ENT/L).glob("*.md")):
        if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", p.stem): continue
        meta, pre, secs = parse(p)
        words = len(re.findall(r"\w+", pre+" ".join(secs.values())))
        out.append(dict(meta=meta, pre=pre, secs=secs, date=datetime.date.fromisoformat(p.stem), slug=p.stem, mins=max(1, round(words/200)),
            title=meta.get("title", p.stem), summary=meta.get("summary",""), ph=meta.get("status","").lower()=="platzhalter"))
    for i,e in enumerate(out): e["no"]=i+1
    return out

def build_lang(L):
    t = T[L]; I = CFG["i18n"][L]
    O = SITE if L=="en" else SITE/"de"
    (O/"tag").mkdir(parents=True, exist_ok=True)
    for old in (O/"tag").glob("*.html"): old.unlink()
    entries = load(L); n = len(entries)
    for i, e in enumerate(entries):
        m = e["meta"]; pre = "../media/" if L=="en" else "../../media/"
        body = [f'<article class="entry"><header class="entry-head"><p class="kicker">{t["day"]} {e["no"]} · {t["fmt"](e["date"],t)}</p><h1>{E(e["title"])}</h1>']
        if e["summary"]: body.append(f'<p class="lede">{inline(e["summary"])}</p>')
        body.append(f'<p class="meta">⏱ {e["mins"]} {t["read"]}' + (f' · {t["work"]}: {E(m["hours"])}' if m.get("hours") else "") + '</p>')
        if e["ph"]: body.append('<div class="ph-banner"><strong>PLACEHOLDER</strong></div>')
        body.append('</header>')
        if m.get("hero"): body.append(media(m.get("hero_caption",""), m["hero"], pre))
        if e["pre"].strip(): body.append(md(e["pre"], pre))
        for s in e["secs"]:
            idx = t["sections"].index(s) if s in t["sections"] else -1
            inner = md(e["secs"][s], pre)
            if idx == 6:
                if CFG.get("repo_public"): inner += f'<p class="code-links"><a class="btn" href="{E(CFG["repo"])}">{t["gh"]}</a></p>'
                else: inner += f'<p class="code-note"><span>{t["notpub"]}</span> {t["pubtxt"]}</p>'
            cls = "sec sec-"+re.sub(r"\W","",["did","mindset","wins","fails","learnings","sources","code"][idx] if idx>=0 else "x")
            body.append(f'<section class="{cls}"><h2><span class="ico">{ICONS[idx] if idx>=0 else "•"}</span>{E(s)}</h2>{inner}</section>')
        pv = entries[i-1] if i else None; nx = entries[i+1] if i<n-1 else None
        nav = '<nav class="pager">' + (f'<a class="prev" href="{pv["slug"]}.html"><small>{t["prev"]}</small><span>{t["day"]} {pv["no"]}: {E(pv["title"])}</span></a>' if pv else '<span></span>') + \
              (f'<a class="next" href="{nx["slug"]}.html"><small>{t["next"]}</small><span>{t["day"]} {nx["no"]}: {E(nx["title"])}</span></a>' if nx else f'<a class="next end" href="../index.html#kapitel"><small>{t["endnote"]}</small><span>{t["back"]}</span></a>') + '</nav>'
        body.append(nav + '</article>')
        (O/"tag"/f'{e["slug"]}.html').write_text(layout(L, f'{t["day"]} {e["no"]}: {e["title"]} – {t["brand"]}', "<main>"+"\n".join(body)+"</main>", 1, "chapters", f'tag/{e["slug"]}.html', e["summary"]), encoding="utf-8")
    status = I["status"]
    stand = "".join(f"<li><span>{E(k)}</span><strong>{E(v)}</strong></li>" for k,v in status.items())
    stand += f'<li><span>{t["entries"]}</span><strong>{n}</strong></li>'
    if n: stand += f'<li><span>{t["last"]}</span><strong>{entries[-1]["date"].strftime("%d.%m.%Y" if L=="de" else "%Y-%m-%d")}</strong></li>'
    prog = CFG["milestones"]; done = sum(1 for x in prog if x["done"]); pct = round(100*done/len(prog)) if prog else 0
    ms = "".join(f'<li class="{"done" if x["done"] else ""}"><span class="chk">{"✓" if x["done"] else ""}</span>{E(x[L])}</li>' for x in prog)
    chapters = "".join(f'<li><a href="tag/{e["slug"]}.html"><span class="no">{t["day"]} {e["no"]}</span><span class="ct"><strong>{E(e["title"])}</strong><small>{t["fmt"](e["date"],t)} · {e["mins"]} {t["min"]}</small><span class="sum">{inline(e["summary"])}</span></span></a></li>' for e in reversed(entries))
    dfmt = lambda d: d.strftime("%d.%m.") if L=="de" else d.strftime("%m/%d")
    tl = "".join(f'<a class="node" href="tag/{e["slug"]}.html"><span class="d"></span><b>{dfmt(e["date"])}</b><small>{t["day"]} {e["no"]}</small></a>' for e in reversed(entries))
    cover = CFG.get("cover","")
    mrel = "" if L=="en" else "../"
    coverhtml = f'<div class="cover-img"><img src="{mrel}media/{cover}" alt=""></div>' if cover and (SITE/"media"/cover).exists() else ""
    first = f'<a class="btn" href="tag/{entries[0]["slug"]}.html">{t["start"]}</a>' if entries else ""
    latest = f'<a class="btn ghost" href="tag/{entries[-1]["slug"]}.html">{t["latest"]}</a>' if entries else ""
    home = f'''<main><section class="cover"><div class="cover-text"><p class="kicker">{t["kicker"]}</p><h1>{E(I["title"])}</h1><p class="lede">{inline(I["intro"])}</p><p class="cta">{first} {latest}</p></div>{coverhtml}</section>
<section class="wrap"><div class="grid2"><div class="card stand"><h2>{t["stand"]}</h2><ul class="facts">{stand}</ul></div>
<div class="card"><h2>{t["progress"]} <small>{pct} %</small></h2><div class="bar"><i style="width:{pct}%"></i></div><ul class="ms">{ms}</ul></div></div>
<h2 id="zeitleiste" class="sh">{t["timeline"]}</h2><div class="timeline">{tl}</div>
<h2 id="kapitel" class="sh">{t["chapters"]}</h2><ol class="chapters">{chapters}</ol></section></main>'''
    (O/"index.html").write_text(layout(L, I["title"], home, 0, "home", "index.html", I["intro"]), encoding="utf-8")
    codeabout = f'<p><a class="btn" href="{E(CFG["repo"])}">{t["ghabout"]}</a></p>' if CFG.get("repo_public") else f'<p class="code-note"><span>{t["notpub"]}</span> {t["pubtxt"]}</p>'
    about = f'''<main class="wrap narrow"><article class="entry"><header class="entry-head"><p class="kicker">{t["about_k"]}</p><h1>{t["about_h"]}</h1></header>
{md(I["about"], "media/")}{codeabout}{contact_block(t)}</article></main>'''
    (O/"about.html").write_text(layout(L, f'{t["page_about"]} – {t["brand"]}', about, 0, "about", "about.html"), encoding="utf-8")
    return n

def main():
    global CFG
    CFG = json.loads((ROOT/"project.json").read_text(encoding="utf-8"))
    SITE.mkdir(exist_ok=True); (SITE/"assets").mkdir(exist_ok=True); (SITE/"media").mkdir(exist_ok=True)
    for f in ASSETS.glob("*"): shutil.copy(f, SITE/"assets"/f.name)
    (SITE/".nojekyll").write_text("")
    for L in ("en","de"): print(L, build_lang(L), "entries")
main()
