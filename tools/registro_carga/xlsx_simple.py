"""
xlsx_simple.py — Escribe un .xlsx con colores y gráficas de líneas usando solo la
librería estándar (el formato es un zip de XML). Suficiente para los informes del
equipo sin instalar openpyxl.

    libro = Libro()
    libro.hoja("Panel", filas, anchos=[(1, 1, 30)], estilos={0: {"*": TITULO}},
               escalas=[("B4:L13", "rojo_verde")], graficas=[Grafica(...)])
    libro.guardar("informe.xlsx")

Estilos de celda: NORMAL, NEGRITA, TRES_DEC (0.000), TITULO, SIGNO (+0;-0), DOS_DEC.
Escalas de color: ver ESCALAS.
"""
import zipfile
from xml.sax.saxutils import escape

NORMAL, NEGRITA, TRES_DEC, TITULO, SIGNO, DOS_DEC = 0, 1, 2, 3, 4, 5
NS = "http://schemas.openxmlformats.org"

ESCALAS = {
    # bajo = rojo, alto = verde (percentiles: resalta los peores del pack)
    "rojo_verde": '<cfvo type="percentile" val="2"/><cfvo type="percentile" val="50"/><cfvo type="max"/>'
                  '<color rgb="FFF8696B"/><color rgb="FFFFEB84"/><color rgb="FF63BE7B"/>',
    # capacidad relativa: 0,7 rojo, 0,9 amarillo, 1,0 verde
    "capacidad": '<cfvo type="num" val="0.7"/><cfvo type="num" val="0.9"/><cfvo type="num" val="1"/>'
                 '<color rgb="FFF8696B"/><color rgb="FFFFEB84"/><color rgb="FF63BE7B"/>',
    # temperatura: 25 azul, 40 amarillo, 60 rojo
    "temperatura": '<cfvo type="num" val="25"/><cfvo type="num" val="40"/><cfvo type="num" val="60"/>'
                   '<color rgb="FF5A8AC6"/><color rgb="FFFFEB84"/><color rgb="FFF8696B"/>',
    # diferencia con la mediana en mV: negativo rojo, 0 blanco, positivo azul
    "diferencia": '<cfvo type="num" val="-100"/><cfvo type="num" val="0"/><cfvo type="num" val="60"/>'
                  '<color rgb="FFC00000"/><color rgb="FFFFFFFF"/><color rgb="FF5A8AC6"/>',
    # alto = rojo (resistencia)
    "verde_rojo": '<cfvo type="min"/><cfvo type="percentile" val="50"/><cfvo type="max"/>'
                  '<color rgb="FF63BE7B"/><color rgb="FFFFEB84"/><color rgb="FFF8696B"/>',
}
COLORES = ["C00000", "FF7F00", "7030A0", "0070C0", "00B050", "808000", "00B0F0", "FF66CC", "996633", "404040"]


def letra(c):
    """Índice de columna (0 = A) a letra."""
    s, c = "", c + 1
    while c:
        c, r = divmod(c - 1, 26)
        s = chr(65 + r) + s
    return s


def rango_col(hoja, c, fila0, fila1):
    """Referencia absoluta a una columna, p.ej. Datos!$B$2:$B$900."""
    return f"'{hoja}'!${letra(c)}${fila0}:${letra(c)}${fila1}"


class Grafica:
    """Gráfica XY de líneas. series: (nombre, ref_x, ref_y, color, grosor_pt, discontinua).
    ancla: (col0, fila0, col1, fila1) en celdas de la hoja donde se dibuja."""

    def __init__(self, titulo, series, ytit, xtit, ancla, ymin=None, ymax=None):
        self.titulo, self.series, self.ytit, self.xtit = titulo, series, ytit, xtit
        self.ancla, self.ymin, self.ymax = ancla, ymin, ymax

    def xml(self):
        def rich(txt, sz):
            return (f'<c:tx><c:rich><a:bodyPr/><a:p><a:pPr><a:defRPr sz="{sz}" b="1"/></a:pPr><a:r><a:rPr lang="es-ES" sz="{sz}" b="1"/>'
                    f'<a:t>{escape(txt)}</a:t></a:r></a:p></c:rich></c:tx><c:overlay val="0"/>')
        sers = []
        for k, (nom, rx, ry, colr, w, dash) in enumerate(self.series):
            dsh = '<a:prstDash val="sysDash"/>' if dash else ""
            sers.append(f'<c:ser><c:idx val="{k}"/><c:order val="{k}"/><c:tx><c:v>{escape(nom)}</c:v></c:tx>'
                        f'<c:spPr><a:ln w="{int(w * 12700)}" cap="rnd"><a:solidFill><a:srgbClr val="{colr}"/></a:solidFill>{dsh}</a:ln></c:spPr>'
                        f'<c:marker><c:symbol val="none"/></c:marker>'
                        f'<c:xVal><c:numRef><c:f>{escape(rx)}</c:f></c:numRef></c:xVal><c:yVal><c:numRef><c:f>{escape(ry)}</c:f></c:numRef></c:yVal>'
                        f'<c:smooth val="0"/></c:ser>')
        sc = ('<c:orientation val="minMax"/>' + (f'<c:max val="{self.ymax}"/>' if self.ymax is not None else "") +
              (f'<c:min val="{self.ymin}"/>' if self.ymin is not None else ""))
        gris = '<c:spPr><a:ln w="6350"><a:solidFill><a:srgbClr val="D9D9D9"/></a:solidFill></a:ln></c:spPr>'
        return ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
                f'<c:chartSpace xmlns:c="{NS}/drawingml/2006/chart" xmlns:a="{NS}/drawingml/2006/main" '
                f'xmlns:r="{NS}/officeDocument/2006/relationships"><c:roundedCorners val="0"/><c:chart>'
                f'<c:title>{rich(self.titulo, 1300)}</c:title><c:autoTitleDeleted val="0"/><c:plotArea><c:layout/>'
                '<c:scatterChart><c:scatterStyle val="lineMarker"/><c:varyColors val="0"/>' + "".join(sers) +
                '<c:axId val="1001"/><c:axId val="1002"/></c:scatterChart>'
                '<c:valAx><c:axId val="1001"/><c:scaling><c:orientation val="minMax"/></c:scaling><c:delete val="0"/><c:axPos val="b"/>'
                f'<c:title>{rich(self.xtit, 900)}</c:title><c:numFmt formatCode="General" sourceLinked="0"/><c:majorTickMark val="out"/>'
                '<c:minorTickMark val="none"/><c:tickLblPos val="low"/><c:crossAx val="1002"/><c:crosses val="autoZero"/><c:crossBetween val="midCat"/></c:valAx>'
                f'<c:valAx><c:axId val="1002"/><c:scaling>{sc}</c:scaling><c:delete val="0"/><c:axPos val="l"/><c:majorGridlines>{gris}</c:majorGridlines>'
                f'<c:title>{rich(self.ytit, 900)}</c:title><c:numFmt formatCode="General" sourceLinked="0"/><c:majorTickMark val="out"/>'
                '<c:minorTickMark val="none"/><c:tickLblPos val="nextTo"/><c:crossAx val="1001"/><c:crosses val="min"/><c:crossBetween val="midCat"/></c:valAx>'
                '</c:plotArea><c:legend><c:legendPos val="r"/><c:overlay val="0"/></c:legend><c:plotVisOnly val="1"/><c:dispBlanksAs val="span"/>'
                '</c:chart></c:chartSpace>')


def _celda(r, c, v, st):
    ref = f"{letra(c)}{r + 1}"
    s = f' s="{st}"' if st else ""
    if v is None or v == "":
        return f'<c r="{ref}"{s}/>' if st else ""
    if isinstance(v, bool):
        v = int(v)
    if isinstance(v, (int, float)):
        return f'<c r="{ref}"{s}><v>{round(v, 5)}</v></c>'
    return f'<c r="{ref}" t="inlineStr"{s}><is><t xml:space="preserve">{escape(str(v))}</t></is></c>'


class Libro:
    def __init__(self):
        self._hojas = []      # (nombre, xml, graficas)

    def hoja(self, nombre, filas, anchos=(), congelar=None, escalas=(), estilos=None, graficas=(), zoom=None):
        """filas: lista de listas. estilos: {fila: {col o '*': estilo}}. congelar: (cols, filas).
        escalas: [(rango 'B2:L11', clave de ESCALAS)]."""
        x = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
             f'<worksheet xmlns="{NS}/spreadsheetml/2006/main" xmlns:r="{NS}/officeDocument/2006/relationships">']
        zv = f' zoomScale="{zoom}"' if zoom else ""
        if congelar:
            c, r = congelar
            x.append(f'<sheetViews><sheetView workbookViewId="0"{zv}><pane xSplit="{c}" ySplit="{r}" '
                     f'topLeftCell="{letra(c)}{r + 1}" activePane="bottomRight" state="frozen"/></sheetView></sheetViews>')
        else:
            x.append(f'<sheetViews><sheetView workbookViewId="0" showGridLines="{0 if graficas else 1}"{zv}/></sheetViews>')
        if anchos:
            x.append("<cols>" + "".join(f'<col min="{a}" max="{b}" width="{w}" customWidth="1"/>' for a, b, w in anchos) + "</cols>")
        x.append("<sheetData>")
        for r, fila in enumerate(filas):
            est = (estilos or {}).get(r, {})
            x.append(f'<row r="{r + 1}">' + "".join(_celda(r, c, v, est.get(c, est.get("*", 0))) for c, v in enumerate(fila)) + "</row>")
        x.append("</sheetData>")
        for p, (rango, tipo) in enumerate(escalas, 1):
            x.append(f'<conditionalFormatting sqref="{rango}"><cfRule type="colorScale" priority="{p}">'
                     f'<colorScale>{ESCALAS[tipo]}</colorScale></cfRule></conditionalFormatting>')
        if graficas:
            x.append('<drawing r:id="rId1"/>')
        x.append("</worksheet>")
        self._hojas.append((nombre, "\n".join(x), list(graficas)))

    def guardar(self, ruta):
        hojas = self._hojas
        ct = [f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="{NS}/package/2006/content-types">'
              '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
              '<Default Extension="xml" ContentType="application/xml"/>'
              '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
              '<Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>']
        partes = {}
        n_graf = n_dib = 0
        for i, (nom, xml, grafs) in enumerate(hojas, 1):
            partes[f"xl/worksheets/sheet{i}.xml"] = xml
            ct.append(f'<Override PartName="/xl/worksheets/sheet{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>')
            if not grafs:
                continue
            n_dib += 1
            partes[f"xl/worksheets/_rels/sheet{i}.xml.rels"] = (
                f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">'
                f'<Relationship Id="rId1" Type="{NS}/officeDocument/2006/relationships/drawing" Target="../drawings/drawing{n_dib}.xml"/></Relationships>')
            ct.append(f'<Override PartName="/xl/drawings/drawing{n_dib}.xml" ContentType="application/vnd.openxmlformats-officedocument.drawing+xml"/>')
            dib = [f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><xdr:wsDr xmlns:xdr="{NS}/drawingml/2006/spreadsheetDrawing" xmlns:a="{NS}/drawingml/2006/main">']
            rels = [f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">']
            for k, g in enumerate(grafs, 1):
                n_graf += 1
                c0, r0, c1, r1 = g.ancla
                dib.append(f'<xdr:twoCellAnchor><xdr:from><xdr:col>{c0}</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>{r0}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:from>'
                           f'<xdr:to><xdr:col>{c1}</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>{r1}</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:to>'
                           f'<xdr:graphicFrame macro=""><xdr:nvGraphicFramePr><xdr:cNvPr id="{k + 1}" name="Grafica {k}"/><xdr:cNvGraphicFramePr/></xdr:nvGraphicFramePr>'
                           '<xdr:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/></xdr:xfrm><a:graphic>'
                           f'<a:graphicData uri="{NS}/drawingml/2006/chart"><c:chart xmlns:c="{NS}/drawingml/2006/chart" '
                           f'xmlns:r="{NS}/officeDocument/2006/relationships" r:id="rId{k}"/></a:graphicData></a:graphic></xdr:graphicFrame>'
                           '<xdr:clientData/></xdr:twoCellAnchor>')
                rels.append(f'<Relationship Id="rId{k}" Type="{NS}/officeDocument/2006/relationships/chart" Target="../charts/chart{n_graf}.xml"/>')
                partes[f"xl/charts/chart{n_graf}.xml"] = g.xml()
                ct.append(f'<Override PartName="/xl/charts/chart{n_graf}.xml" ContentType="application/vnd.openxmlformats-officedocument.drawingml.chart+xml"/>')
            partes[f"xl/drawings/drawing{n_dib}.xml"] = "".join(dib) + "</xdr:wsDr>"
            partes[f"xl/drawings/_rels/drawing{n_dib}.xml.rels"] = "".join(rels) + "</Relationships>"
        ct.append("</Types>")
        partes["[Content_Types].xml"] = "".join(ct)
        partes["_rels/.rels"] = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">'
                                 f'<Relationship Id="rId1" Type="{NS}/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>')
        partes["xl/workbook.xml"] = (f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><workbook xmlns="{NS}/spreadsheetml/2006/main" '
                                     f'xmlns:r="{NS}/officeDocument/2006/relationships"><sheets>' +
                                     "".join(f'<sheet name="{escape(n)}" sheetId="{i}" r:id="rId{i}"/>' for i, (n, _, _) in enumerate(hojas, 1)) +
                                     "</sheets></workbook>")
        partes["xl/_rels/workbook.xml.rels"] = (
            f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="{NS}/package/2006/relationships">' +
            "".join(f'<Relationship Id="rId{i}" Type="{NS}/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{i}.xml"/>'
                    for i in range(1, len(hojas) + 1)) +
            f'<Relationship Id="rId{len(hojas) + 1}" Type="{NS}/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>')
        partes["xl/styles.xml"] = (
            f'<?xml version="1.0" encoding="UTF-8" standalone="yes"?><styleSheet xmlns="{NS}/spreadsheetml/2006/main">'
            '<numFmts count="3"><numFmt numFmtId="164" formatCode="0.000"/><numFmt numFmtId="165" formatCode="+0;-0;0"/>'
            '<numFmt numFmtId="166" formatCode="0.00"/></numFmts>'
            '<fonts count="3"><font><sz val="10"/><name val="Calibri"/></font><font><b/><sz val="10"/><name val="Calibri"/></font>'
            '<font><b/><sz val="14"/><color rgb="FF1F3864"/><name val="Calibri"/></font></fonts>'
            '<fills count="2"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill></fills>'
            '<borders count="1"><border/></borders><cellStyleXfs count="1"><xf/></cellStyleXfs>'
            '<cellXfs count="6"><xf fontId="0"/><xf fontId="1" applyFont="1"/><xf fontId="0" numFmtId="164" applyNumberFormat="1"/>'
            '<xf fontId="2" applyFont="1"/><xf fontId="0" numFmtId="165" applyNumberFormat="1"/>'
            '<xf fontId="0" numFmtId="166" applyNumberFormat="1"/></cellXfs></styleSheet>')
        with zipfile.ZipFile(ruta, "w", zipfile.ZIP_DEFLATED) as z:
            for nombre, contenido in partes.items():
                z.writestr(nombre, contenido)
        return ruta
