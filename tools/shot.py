from playwright.sync_api import sync_playwright
S="/workspace/humanoid-log/screens/"
jobs=[]
for lang,pre in (("en",""),("de","de/")):
    jobs+=[(1280,f"{lang}-landing",pre+"index.html"),(1280,f"{lang}-entry-2026-11-15",pre+"tag/2026-11-15.html"),(1280,f"{lang}-entry-2026-11-16",pre+"tag/2026-11-16.html"),(1280,f"{lang}-entry-2026-11-21",pre+"tag/2026-11-21.html"),(390,f"{lang}-landing-mobile",pre+"index.html"),(390,f"{lang}-entry-mobile",pre+"tag/2026-11-21.html")]
with sync_playwright() as p:
    b=p.chromium.launch()
    for w,n,u in jobs:
        pg=b.new_page(viewport={"width":w,"height":900}); pg.goto("http://127.0.0.1:8091/"+u); pg.wait_for_timeout(400)
        pg.screenshot(path=S+n+".png",full_page=True)
    b.close()
