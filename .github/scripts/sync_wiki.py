#!/usr/bin/env python3
"""
Sync documentation from lekiwi_ros2/docs to GitHub Wiki (lekiwi_ros2.wiki).

Why Flattening is Required for GitHub Wiki:
GitHub Wiki Web UI only supports a flat page namespace (https://github.com/<user>/<repo>/wiki/<Page-Name>).
Subdirectories (e.g. /tutorials/01.md) cause GitHub's router to treat links as static attachments,
redirecting users to 'raw.githubusercontent.com' and triggering 404 Not Found errors.

Tasks performed:
1. Build a path-to-slug mapping dictionary for all documents.
2. Flatten all Markdown files directly into the root directory of the Wiki repo.
3. Transform root 'index.md' into 'Home.md' (GitHub Wiki landing page).
4. Rewrite all internal Markdown links to point to the corresponding flat Wiki slugs (without .md).
5. Dynamically construct a Diátaxis-compliant '_Sidebar.md' linking to flattened page slugs.
6. Generate standard '_Footer.md' linking back to the source repository.
"""

import argparse
import os
import re
import shutil
import sys
from pathlib import Path


def get_wiki_slug(rel_path: Path) -> str:
    """Compute a clean, unique flat slug for a Markdown file in the Wiki."""
    parts = list(rel_path.parts)
    fname = rel_path.name

    if fname == "index.md":
        if len(parts) == 1:
            return "Home"
        parent = parts[-2]
        return f"{parent.replace('-', ' ').title().replace(' ', '-')}-Overview"

    stem = rel_path.stem
    if stem in ["actions", "services", "topics-and-tf"]:
        return f"ros-interfaces-{stem}"
    if stem == "catalog":
        return "packages-catalog"

    return stem


def build_mapping(docs_dir: Path) -> dict:
    """Build a dictionary mapping relative docs paths to Wiki slugs."""
    mapping = {}
    for p in docs_dir.rglob("*.md"):
        rel = p.relative_to(docs_dir)
        norm_rel = str(rel).replace("\\", "/")
        mapping[norm_rel] = get_wiki_slug(rel)
    return mapping


def resolve_wiki_link(source_rel: str, link_target: str, mapping: dict) -> str:
    """Resolve an internal Markdown link to its corresponding flat Wiki slug."""
    if link_target.startswith(("http://", "https://", "mailto:", "#")):
        return link_target

    anchor = ""
    if "#" in link_target:
        link_target, anchor = link_target.split("#", 1)
        anchor = f"#{anchor}"

    source_dir = Path(source_rel).parent
    target_norm = os.path.normpath(source_dir / link_target).replace("\\", "/")

    if target_norm in mapping:
        return mapping[target_norm] + anchor
    if f"{target_norm}.md" in mapping:
        return mapping[f"{target_norm}.md"] + anchor

    # Fallback: strip .md if present
    clean_name = re.sub(r"\.md$", "", Path(link_target).name)
    return clean_name + anchor


def transform_markdown_for_wiki(content: str, source_rel: str, mapping: dict) -> str:
    """Sanitize MkDocs tags and rewrite all internal links to flat Wiki slugs."""
    # Replace Material & Octicon icons with unicode equivalents
    icon_map = {
        r":material-school:\{[^}]*\}": "🎓",
        r":material-wrench:\{[^}]*\}": "🛠️",
        r":material-book-open-page-variant:\{[^}]*\}": "📖",
        r":material-lightbulb-on:\{[^}]*\}": "💡",
        r":octicons-arrow-right-24:": "➡️",
        r":octicons-check-circle-fill-24:": "✅",
        r":octicons-alert-fill-24:": "⚠️",
    }
    for pattern, replacement in icon_map.items():
        content = re.sub(pattern, replacement, content)

    # Clean empty or leftover grid container divs
    content = re.sub(r'<div class="grid cards"[^>]*>', "", content)
    content = content.replace("</div>", "")

    # Rewrite internal Markdown links: [Text](path/to/page.md) -> [Text](Wiki-Slug)
    def link_replacer(match):
        text = match.group(1)
        raw_target = match.group(2)
        new_slug = resolve_wiki_link(source_rel, raw_target, mapping)
        return f"[{text}]({new_slug})"

    link_pattern = r"\[([^\]]+)\]\(((?!https?:\/\/|mailto:)[^)]+)\)"
    content = re.sub(link_pattern, link_replacer, content)

    return content


def parse_index_file(index_file: Path, folder_prefix: str, mapping: dict) -> list:
    """Dynamically parse headings, groups, and markdown links from an index.md file."""
    if not index_file.exists():
        return []

    lines = index_file.read_text(encoding="utf-8").splitlines()
    entries = []
    current_group = None
    source_rel = str(index_file.relative_to(index_file.parents[len(Path(folder_prefix).parts)]))

    for line in lines:
        s = line.strip()

        # Match section headers with link: ### 1. [Title](target.md)
        h_link = re.match(r"^###\s+(\d+\.\s*)?\[\*{0,2}(.*?)\*{0,2}\]\((.*?\.md)\)", s)
        if h_link:
            prefix = h_link.group(1) or ""
            title = prefix + h_link.group(2).strip()
            target = h_link.group(3).strip()
            slug = resolve_wiki_link(source_rel, target, mapping)
            entries.append({"group": current_group, "title": title, "slug": slug})
            continue

        # Match section subheadings without link
        h_group = re.match(r"^###\s+(?!\[)(.+)$", s)
        if h_group:
            current_group = h_group.group(1).strip()
            continue

        # Match list items with links: - [Title](target.md)
        list_link = re.match(r"^[-*]\s+\[\*{0,2}(.*?)\*{0,2}\]\((.*?\.md)\)", s)
        if list_link:
            title = list_link.group(1).strip()
            target = list_link.group(2).strip()
            slug = resolve_wiki_link(source_rel, target, mapping)
            entries.append({"group": current_group, "title": title, "slug": slug})
            continue

        # Match ADR table rows
        adr_match = re.search(r"\|\s*\[\*{0,2}(ADR-[^\]]+?)\*{0,2}\]\((.*?\.md)\)\s*\|\s*([^\|]+?)\s*\|", s)
        if adr_match:
            adr_id = adr_match.group(1).strip()
            target = adr_match.group(2).strip()
            desc = adr_match.group(3).strip()
            title = f"{adr_id}: {desc}"
            slug = resolve_wiki_link(source_rel, target, mapping)
            entries.append({"group": current_group, "title": title, "slug": slug})
            continue

    return entries


def generate_dynamic_sidebar(docs_dir: Path, mapping: dict) -> str:
    """Dynamically generate Diátaxis-compliant GitHub Wiki sidebar navigation."""
    categories = [
        ("tutorials", "🎓", "Tutorials", "Learning Overview"),
        ("how-to-guides", "🛠️", "How-To Guides", "Task Catalog"),
        ("reference", "📖", "Reference", "Reference Overview"),
        ("explanation", "💡", "Architecture & Explanation", "Architecture Overview"),
    ]

    out = ["<!-- Auto-generated by sync_wiki.py - DO NOT EDIT MANUALLY -->"]
    out.append("### 🥝 [LeKiwi Documentation](Home)\n")

    for cat_dir, icon, cat_title, overview_title in categories:
        index_path = docs_dir / cat_dir / "index.md"
        overview_slug = mapping.get(f"{cat_dir}/index.md", f"{cat_title}-Overview")

        out.append("---")
        out.append(f"#### {icon} {cat_title}")
        out.append(f"- [🧭 {overview_title}]({overview_slug})")

        entries = parse_index_file(index_path, cat_dir, mapping)

        # Inspect nested ADR index for Explanation category
        if cat_dir == "explanation":
            adr_index = docs_dir / "explanation" / "adr" / "index.md"
            if adr_index.exists():
                adr_entries = parse_index_file(adr_index, "explanation/adr", mapping)
                for e in adr_entries:
                    e["group"] = "Architecture Decision Records (ADRs)"
                    entries.append(e)

        # Group entries by subsection
        grouped = {}
        for e in entries:
            grp = e["group"]
            if grp not in grouped:
                grouped[grp] = []
            grouped[grp].append(e)

        for grp, items in grouped.items():
            if grp:
                out.append(f"- **{grp}**")
                for it in items:
                    out.append(f"  - [{it['title']}]({it['slug']})")
            else:
                for it in items:
                    out.append(f"- [{it['title']}]({it['slug']})")
        out.append("")

    out.append("---")
    out.append("[🏠 Return to Home](Home)\n")
    return "\n".join(out)


def generate_footer_content() -> str:
    """Generate standard GitHub Wiki footer."""
    return """<!-- Auto-generated by sync_wiki.py - DO NOT EDIT MANUALLY -->
---
*LeKiwi ROS 2 Platform Technical Documentation • Synced from [lekiwi_ros2 repository](https://github.com/DuyKhongCay/lekiwi_ros2)*
"""


def sync_docs(docs_dir: Path, wiki_dir: Path) -> None:
    """Sync and flatten docs directory into root of wiki directory."""
    if not docs_dir.exists():
        raise FileNotFoundError(f"Source docs directory not found: {docs_dir}")
    if not wiki_dir.exists():
        raise FileNotFoundError(f"Target wiki directory not found: {wiki_dir}")

    print(f"🔄 Syncing & flattening docs from '{docs_dir}' to '{wiki_dir}'...")

    # 1. Clean existing files in wiki_dir except .git and .gitignore
    for item in wiki_dir.iterdir():
        if item.name in [".git", ".gitignore"]:
            continue
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()

    # 2. Build mapping of all Markdown files to flat Wiki slugs
    mapping = build_mapping(docs_dir)

    # 3. Copy & Flatten Markdown files to the root of wiki_dir
    for src_path in docs_dir.rglob("*.md"):
        rel_str = str(src_path.relative_to(docs_dir)).replace("\\", "/")
        slug = mapping[rel_str]
        target_name = "Home.md" if slug == "Home" else f"{slug}.md"
        dest_path = wiki_dir / target_name

        content = src_path.read_text(encoding="utf-8")
        transformed = transform_markdown_for_wiki(content, rel_str, mapping)
        dest_path.write_text(transformed, encoding="utf-8")

    print(f"  ✅ Flattened {len(mapping)} pages into Wiki root directory.")

    # 4. Copy static asset directories (images, assets) if they exist
    for asset_dir_name in ["assets", "images"]:
        asset_src = docs_dir / asset_dir_name
        if asset_src.exists() and asset_src.is_dir():
            shutil.copytree(asset_src, wiki_dir / asset_dir_name)
            print(f"  ✅ Copied static asset directory '{asset_dir_name}'.")

    # 5. Generate _Sidebar.md with flat slugs
    sidebar_file = wiki_dir / "_Sidebar.md"
    sidebar_content = generate_dynamic_sidebar(docs_dir, mapping)
    sidebar_file.write_text(sidebar_content, encoding="utf-8")
    print("  ✅ Generated flat-slug '_Sidebar.md'")

    # 6. Generate _Footer.md
    footer_file = wiki_dir / "_Footer.md"
    footer_file.write_text(generate_footer_content(), encoding="utf-8")
    print("  ✅ Generated '_Footer.md'")

    print("🎉 Flat-Wiki Sync completed successfully!")


def main():
    parser = argparse.ArgumentParser(description="Sync and flatten LeKiwi docs for GitHub Wiki")
    
    repo_root = Path(__file__).resolve().parents[2]
    ws_root = repo_root.parent

    parser.add_argument(
        "--docs-dir",
        type=Path,
        default=repo_root / "docs",
        help="Path to source docs directory",
    )
    parser.add_argument(
        "--wiki-dir",
        type=Path,
        default=ws_root / "lekiwi_ros2.wiki",
        help="Path to target GitHub Wiki directory",
    )
    args = parser.parse_args()

    try:
        sync_docs(args.docs_dir.resolve(), args.wiki_dir.resolve())
    except Exception as e:
        print(f"❌ Error during sync: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
