#!/usr/bin/env python3
"""
Extract tables and technical specifications from i.MX 93 datasheet using vision LLM.

Usage:
    python scripts/extract_imx93_datasheet.py --test-page 10
    python scripts/extract_imx93_datasheet.py --batch --start-page 1 --end-page 109
"""

import argparse
import base64
import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional

import requests
from pdf2image import convert_from_path

# Configuration
OLLAMA_HOST = os.getenv('OLLAMA_HOST', 'http://rig.localnet:11434')
MODEL_NAME = 'qwen2.5vl:32b'  # Same model that worked well for SAAMI
PDF_PATH = '/Users/alexjokela/projects/imx93/dc96c1a2-51aa-40c0-84ae-ef6e7d31713a.pdf'
OUTPUT_DIR = Path('/Users/alexjokela/projects/imx93/data/imx93_pages')
OUTPUT_JSON = Path('/Users/alexjokela/projects/imx93/data/imx93_extraction.json')

# Vision LLM prompt for electronics datasheet extraction
EXTRACTION_PROMPT = """You are analyzing a page from the i.MX 93 Applications Processor datasheet.

Extract ALL technical information from this page in a structured format. Pay special attention to:

1. **Tables**: Extract complete tables with headers and all rows/columns
2. **Specifications**: Electrical characteristics, timing parameters, operating conditions
3. **Register Descriptions**: Memory maps, bit fields, register addresses
4. **Pin Configurations**: Pin names, functions, ball numbers
5. **Block Diagrams**: Component names and connections (describe textually)
6. **Formulas**: Mathematical equations and relationships
7. **Notes and Footnotes**: Important clarifications

For TABLES, use this format:
{
  "table_title": "Table name from document",
  "table_number": "Table X-Y if present",
  "headers": ["Column 1", "Column 2", ...],
  "rows": [
    ["value1", "value2", ...],
    ["value1", "value2", ...]
  ]
}

For SPECIFICATIONS, use this format:
{
  "parameter": "Parameter name",
  "symbol": "Symbol if present",
  "min": "Minimum value",
  "typ": "Typical value",
  "max": "Maximum value",
  "unit": "Unit of measurement",
  "conditions": "Test conditions"
}

For REGISTERS, use this format:
{
  "register_name": "Register name",
  "address": "Hex address",
  "bits": [
    {"bit_range": "31:24", "field_name": "Field name", "access": "RW", "description": "Field description"},
    ...
  ]
}

Return a JSON object with these top-level keys:
- "page_number": integer
- "section_title": "Section heading if visible"
- "tables": [array of table objects]
- "specifications": [array of spec objects]
- "registers": [array of register objects]
- "text_content": "Key textual information not in tables"
- "notes": [array of important notes/footnotes]

IMPORTANT:
- Extract EXACT values from the document (do not estimate or round)
- Preserve units exactly as shown
- Include all footnote references
- If a table spans multiple pages, note "CONTINUED FROM PREVIOUS PAGE" or "CONTINUES ON NEXT PAGE"
- Return ONLY valid JSON, no explanatory text

If the page contains primarily text (no tables/specs), extract the key technical information in "text_content" field.
"""


def convert_pdf_to_images(pdf_path: str, start_page: int = 1, end_page: int = 109, dpi: int = 300) -> List[Path]:
    """Convert PDF pages to PNG images at specified DPI."""
    print(f"Converting PDF pages {start_page}-{end_page} to PNG at {dpi} DPI...")

    images = convert_from_path(
        pdf_path,
        dpi=dpi,
        first_page=start_page,
        last_page=end_page,
        fmt='png'
    )

    image_paths = []
    for i, image in enumerate(images, start=start_page):
        image_path = OUTPUT_DIR / f'imx93_page_{i:03d}.png'
        image.save(image_path, 'PNG')
        image_paths.append(image_path)
        print(f"  Saved: {image_path.name}")

    return image_paths


def encode_image_base64(image_path: Path) -> str:
    """Encode image to base64 string."""
    with open(image_path, 'rb') as f:
        return base64.b64encode(f.read()).decode('utf-8')


def extract_page_with_vision_llm(image_path: Path, page_number: int) -> Optional[Dict]:
    """Extract data from a single page using vision LLM."""
    print(f"\n{'='*80}")
    print(f"Processing page {page_number}: {image_path.name}")
    print(f"{'='*80}")

    # Encode image
    image_b64 = encode_image_base64(image_path)

    # Call vision LLM
    payload = {
        'model': MODEL_NAME,
        'prompt': EXTRACTION_PROMPT,
        'images': [image_b64],
        'stream': False,
        'options': {
            'temperature': 0.1,  # Low temperature for consistent extraction
            'num_predict': 4096   # Allow long outputs for complex tables
        }
    }

    try:
        response = requests.post(
            f'{OLLAMA_HOST}/api/generate',
            json=payload,
            timeout=300  # 5 minute timeout for complex pages
        )
        response.raise_for_status()

        result = response.json()
        llm_output = result.get('response', '')

        # Parse JSON from LLM output
        # Sometimes LLM wraps JSON in markdown code blocks
        if '```json' in llm_output:
            llm_output = llm_output.split('```json')[1].split('```')[0].strip()
        elif '```' in llm_output:
            llm_output = llm_output.split('```')[1].split('```')[0].strip()

        try:
            data = json.loads(llm_output)
            data['page_number'] = page_number
            data['source_file'] = 'dc96c1a2-51aa-40c0-84ae-ef6e7d31713a.pdf'

            # Print summary
            print(f"✅ Extracted from page {page_number}:")
            print(f"   - Tables: {len(data.get('tables', []))}")
            print(f"   - Specifications: {len(data.get('specifications', []))}")
            print(f"   - Registers: {len(data.get('registers', []))}")
            print(f"   - Notes: {len(data.get('notes', []))}")
            if data.get('section_title'):
                print(f"   - Section: {data['section_title']}")

            return data

        except json.JSONDecodeError as e:
            print(f"❌ JSON parsing error on page {page_number}: {e}")
            print(f"   Raw output: {llm_output[:200]}...")
            return None

    except requests.exceptions.RequestException as e:
        print(f"❌ API error on page {page_number}: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(description='Extract i.MX 93 datasheet tables and specs')
    parser.add_argument('--test-page', type=int, help='Test extraction on a single page')
    parser.add_argument('--batch', action='store_true', help='Process multiple pages')
    parser.add_argument('--start-page', type=int, default=1, help='Starting page number')
    parser.add_argument('--end-page', type=int, default=109, help='Ending page number')
    parser.add_argument('--dpi', type=int, default=300, help='Image resolution (default: 300)')
    parser.add_argument('--output', type=str, help='Output JSON file path')
    parser.add_argument('--skip-convert', action='store_true', help='Skip PDF conversion (use existing PNGs)')

    args = parser.parse_args()

    # Determine output path
    output_path = Path(args.output) if args.output else OUTPUT_JSON

    # Test mode: single page
    if args.test_page:
        print(f"\n🔬 TEST MODE: Extracting page {args.test_page}\n")

        if not args.skip_convert:
            image_paths = convert_pdf_to_images(PDF_PATH, args.test_page, args.test_page, args.dpi)
        else:
            image_paths = [OUTPUT_DIR / f'imx93_page_{args.test_page:03d}.png']

        data = extract_page_with_vision_llm(image_paths[0], args.test_page)

        if data:
            print(f"\n✅ Extraction successful!")
            print(f"\nExtracted data preview:")
            print(json.dumps(data, indent=2)[:1000] + "...\n")
        else:
            print(f"\n❌ Extraction failed")
            sys.exit(1)

    # Batch mode: multiple pages
    elif args.batch:
        print(f"\n📚 BATCH MODE: Extracting pages {args.start_page}-{args.end_page}\n")

        if not args.skip_convert:
            image_paths = convert_pdf_to_images(PDF_PATH, args.start_page, args.end_page, args.dpi)
        else:
            image_paths = [
                OUTPUT_DIR / f'imx93_page_{i:03d}.png'
                for i in range(args.start_page, args.end_page + 1)
            ]

        all_data = []
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Create checkpoint file for progress tracking
        checkpoint_path = output_path.with_suffix('.checkpoint.json')

        for i, image_path in enumerate(image_paths, start=args.start_page):
            data = extract_page_with_vision_llm(image_path, i)
            if data:
                all_data.append(data)

                # Save checkpoint after each page (incremental progress)
                try:
                    with open(checkpoint_path, 'w') as f:
                        json.dump({
                            'last_completed_page': i,
                            'total_pages': args.end_page,
                            'pages_completed': len(all_data),
                            'progress_percent': round(len(all_data) * 100 / len(image_paths), 1),
                            'tables_extracted': sum(len(d.get('tables', [])) for d in all_data),
                            'specs_extracted': sum(len(d.get('specifications', [])) for d in all_data),
                            'registers_extracted': sum(len(d.get('registers', [])) for d in all_data)
                        }, f, indent=2)
                    # Flush stdout to see print statements immediately
                    sys.stdout.flush()
                except Exception as e:
                    print(f"Warning: Could not save checkpoint: {e}")

        # Save final results
        with open(output_path, 'w') as f:
            json.dump(all_data, f, indent=2)

        # Remove checkpoint file
        if checkpoint_path.exists():
            checkpoint_path.unlink()

        print(f"\n{'='*80}")
        print(f"✅ Batch extraction complete!")
        print(f"   - Pages processed: {len(all_data)}/{len(image_paths)}")
        print(f"   - Output saved to: {output_path}")
        print(f"   - Total tables: {sum(len(d.get('tables', [])) for d in all_data)}")
        print(f"   - Total specs: {sum(len(d.get('specifications', [])) for d in all_data)}")
        print(f"   - Total registers: {sum(len(d.get('registers', [])) for d in all_data)}")
        print(f"{'='*80}\n")

    else:
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main()
