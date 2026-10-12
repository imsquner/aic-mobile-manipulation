import { createRequire } from "node:module";
import { join } from "node:path";

const require = createRequire(join(process.cwd(), "package.json"));
const { chromium } = require("playwright");

const [outputPath, targetUrl] = process.argv.slice(2);
if (!outputPath) {
  throw new Error("usage: node capture_vm_chromium.mjs <output.png>");
}

const browser = await chromium.connectOverCDP("http://127.0.0.1:9222");
const pages = browser.contexts().flatMap((context) => context.pages());
const page = pages.find((candidate) => candidate.url().includes("127.0.0.1:8088")) ?? pages[0];
if (!page) {
  throw new Error("no Chromium page found");
}

if (targetUrl) {
  if (targetUrl === "rosbridge-reset") {
    await page.evaluate(() => localStorage.removeItem("studio.layout"));
  }
  const url =
    targetUrl === "live"
      ? "http://127.0.0.1:8088/?ds=foxglove-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A8765"
      : targetUrl === "rosbridge" || targetUrl === "rosbridge-reset"
        ? "http://127.0.0.1:8088/?ds=rosbridge-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A9090"
      : targetUrl;
  await page.goto(url, { waitUntil: "domcontentloaded" });
}
await page.waitForTimeout(3000);
await page.screenshot({ path: outputPath });
const evidence = await page.evaluate(() => ({
  url: location.href,
  title: document.title,
  rootChildren: document.querySelector("#root")?.childElementCount ?? 0,
  bodyText: document.body.innerText.slice(0, 500),
}));
console.log(JSON.stringify(evidence));
process.exit(0);
