#!/usr/bin/env python3

"""
Publica docs/wiki/ en la wiki de GitHub del repositorio.

La wiki de GitHub vive en otro repositorio (`<repo>.wiki.git`) y no entiende
los enlaces tal como se escriben en docs/wiki/:

- sus paginas no llevan extension: `Simulation.md` da 404, hay que enlazar
  al nombre de la pagina;
- sus nombres de pagina son los de este script, en espanol, no los de los
  ficheros;
- no puede resolver rutas relativas al codigo (`../../src/...`), porque el
  codigo esta en otro repositorio;
- muestra el nombre de la pagina como titulo, asi que el `#` inicial de cada
  fichero saldria repetido.

Este script hace esas cuatro conversiones y sube el resultado. La wiki no se
edita nunca a mano: se cambia docs/wiki/ y se vuelve a publicar.

Uso:
    python3 tools/publicar-wiki.py --dry-run      # solo genera y avisa
    python3 tools/publicar-wiki.py                # genera, commitea y sube
    python3 tools/publicar-wiki.py --salida DIR   # deja el resultado en DIR
"""

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
ORIGEN = RAIZ / 'docs' / 'wiki'

# Nombre de cada pagina en la wiki. La clave es el fichero de docs/wiki/.
PAGINAS = {
    'Home.md': 'Home',
    'Installation-and-build.md': 'Instalación y compilación',
    'Simulation.md': 'Ejecución de la simulación',
    'Architecture.md': 'Arquitectura y flujo de datos',
    'Nodes-and-topics.md': 'Nodos y tópicos',
    'Guidance-modes.md': 'Modos de guiado',
    'Code-walkthrough.md': 'Lectura guiada del código',
    'Line-by-line-project-and-build-files.md': 'Proyecto y construcción línea por línea',
    'Line-by-line-odometry-nodes.md': 'Nodos de odometría línea por línea',
    'Line-by-line-guidance-modes.md': 'Modos de guiado línea por línea',
    'Workspace-file-map.md': 'Mapa del workspace y dependencias',
    'Quick-reference-and-troubleshooting.md': 'Referencia rápida y solución de problemas',
}

# Orden del recorrido recomendado, para la barra lateral.
RECORRIDO = [
    'Home.md',
    'Installation-and-build.md',
    'Simulation.md',
    'Architecture.md',
    'Nodes-and-topics.md',
    'Guidance-modes.md',
    'Code-walkthrough.md',
    'Line-by-line-project-and-build-files.md',
    'Line-by-line-odometry-nodes.md',
    'Line-by-line-guidance-modes.md',
    'Workspace-file-map.md',
    'Quick-reference-and-troubleshooting.md',
]

ENLACE_PAGINA = re.compile(r'\]\(([A-Za-z0-9._-]+\.md)(#[^)]*)?\)')
ENLACE_REPO = re.compile(r'\]\(\.\./\.\./([^)]+)\)')
TITULO = re.compile(r'\A#\s+[^\n]*\n+')


def slug(nombre_pagina):
    """El nombre de una pagina en una URL: los espacios pasan a guiones."""
    return nombre_pagina.replace(' ', '-')


def url_repositorio():
    url = subprocess.run(
        ['git', 'remote', 'get-url', 'origin'],
        cwd=RAIZ, capture_output=True, text=True, check=True).stdout.strip()
    if url.startswith('git@github.com:'):
        url = 'https://github.com/' + url[len('git@github.com:'):]
    return url[:-len('.git')] if url.endswith('.git') else url


def convertir(texto, fichero, base_repo, avisos):
    def pagina(m):
        destino, ancla = m.group(1), m.group(2) or ''
        if destino not in PAGINAS:
            avisos.append(f'{fichero}: enlaza a {destino}, que no es una página de la wiki')
            return m.group(0)
        return f']({slug(PAGINAS[destino])}{ancla})'

    def repo(m):
        ruta = m.group(1)
        destino = RAIZ / ruta
        if not destino.exists():
            avisos.append(f'{fichero}: enlaza a {ruta}, que no existe en el repositorio')
        tipo = 'tree' if destino.is_dir() else 'blob'
        return f']({base_repo}/{tipo}/main/{ruta})'

    texto = ENLACE_PAGINA.sub(pagina, texto)
    texto = ENLACE_REPO.sub(repo, texto)
    # GitHub ya muestra el nombre de la pagina como titulo.
    return TITULO.sub('', texto, count=1)


def barra_lateral():
    lineas = ['### Recorrido', '']
    for fichero in RECORRIDO:
        nombre = PAGINAS[fichero]
        lineas.append(f'- [{nombre}]({slug(nombre)})')
    return '\n'.join(lineas) + '\n'


def generar(destino, base_repo):
    avisos = []
    ficheros = sorted(p.name for p in ORIGEN.glob('*.md'))
    faltan = [f for f in ficheros if f not in PAGINAS]
    if faltan:
        sys.exit(f'Estos ficheros de docs/wiki/ no tienen nombre de página en PAGINAS: {faltan}')

    for fichero in ficheros:
        texto = (ORIGEN / fichero).read_text(encoding='utf-8')
        salida = destino / f'{slug(PAGINAS[fichero])}.md'
        salida.write_text(convertir(texto, fichero, base_repo, avisos), encoding='utf-8')
    (destino / '_Sidebar.md').write_text(barra_lateral(), encoding='utf-8')
    return ficheros, avisos


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dry-run', action='store_true',
                        help='genera las páginas y enseña el diff, sin subir nada')
    parser.add_argument('--salida', metavar='DIR',
                        help='deja las páginas generadas en DIR en vez de publicarlas')
    args = parser.parse_args()

    base_repo = url_repositorio()

    if args.salida:
        destino = Path(args.salida).resolve()
        destino.mkdir(parents=True, exist_ok=True)
        ficheros, avisos = generar(destino, base_repo)
        for aviso in avisos:
            print(f'AVISO  {aviso}')
        print(f'{len(ficheros)} páginas generadas en {destino}')
        return

    temporal = Path(tempfile.mkdtemp(prefix='wiki-'))
    clon = temporal / 'wiki'
    try:
        subprocess.run(['git', 'clone', '--quiet', f'{base_repo}.wiki.git', str(clon)], check=True)
        for previo in clon.glob('*.md'):
            previo.unlink()
        ficheros, avisos = generar(clon, base_repo)
        for aviso in avisos:
            print(f'AVISO  {aviso}')

        subprocess.run(['git', 'add', '-A'], cwd=clon, check=True)
        estado = subprocess.run(['git', 'status', '--porcelain'],
                                cwd=clon, capture_output=True, text=True, check=True).stdout
        if not estado.strip():
            print('La wiki ya está al día; no hay nada que subir.')
            return

        print(subprocess.run(['git', 'diff', '--cached', '--stat'],
                             cwd=clon, capture_output=True, text=True, check=True).stdout)
        if args.dry_run:
            print(f'--dry-run: no se sube nada. Resultado en {clon}')
            temporal = None  # no borrarlo, para poder mirarlo
            return

        commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                                cwd=RAIZ, capture_output=True, text=True, check=True).stdout.strip()
        subprocess.run(['git', 'commit', '--quiet', '-m',
                        f'Publicar docs/wiki/ desde {commit}'], cwd=clon, check=True)
        subprocess.run(['git', 'push', '--quiet'], cwd=clon, check=True)
        print(f'{len(ficheros)} páginas publicadas en {base_repo}/wiki')
    finally:
        if temporal is not None:
            shutil.rmtree(temporal, ignore_errors=True)


if __name__ == '__main__':
    main()
