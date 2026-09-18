import cairosvg
o=[]
def L(*pts, w=2):
    d=" ".join(f"{x},{y}" for x,y in pts)
    o.append(f'<polyline points="{d}" fill="none" stroke="#111" stroke-width="{w}" stroke-linejoin="round" stroke-linecap="round"/>')
def dot(x,y): o.append(f'<circle cx="{x}" cy="{y}" r="3.6" fill="#111"/>')
def term(x,y): o.append(f'<circle cx="{x}" cy="{y}" r="4.5" fill="#fff" stroke="#111" stroke-width="2"/>')
def T(x,y,s,a="start",size=14,fill="#111",weight="normal",style="normal"):
    o.append(f'<text x="{x}" y="{y}" text-anchor="{a}" font-size="{size}" fill="{fill}" font-weight="{weight}" font-style="{style}">{s}</text>')
def note(x,y,s,a="start"): T(x,y,s,a,12,"#666",style="italic")
def pin(x,y,s,a="start"): T(x,y,s,a,11,"#555")
def rh(x,y):  # horizontal resistor, 60 long
    L((x,y),(x+5,y-8),(x+15,y+8),(x+25,y-8),(x+35,y+8),(x+45,y-8),(x+55,y+8),(x+60,y))
def rv(x,y):
    L((x,y),(x-8,y+5),(x+8,y+15),(x-8,y+25),(x+8,y+35),(x-8,y+45),(x+8,y+55),(x,y+60))
def gnd(x,y):
    L((x-14,y),(x+14,y)); L((x-9,y+6),(x+9,y+6)); L((x-4,y+12),(x+4,y+12))
def vcc(x,y):
    L((x-18,y),(x+18,y)); T(x,y-9,"+5 V","middle")
def cap(x,y):  # plates at y and y+10
    L((x-18,y),(x+18,y)); L((x-18,y+10),(x+18,y+10))
def opamp(x,y,name,pp,pm,po):  # x=left edge, y=centre; + at y-25, - at y+25
    L((x,y-50),(x+90,y),(x,y+50),(x,y-50))
    T(x+9,y-20,"+",size=16); T(x+9,y+30,"−",size=16)
    T(x+42,y-40,name,weight="bold")
    pin(x-6,y-31,pp,"end"); pin(x-6,y+19,pm,"end"); pin(x+96,y-6,po)
    # follower loop
    L((x+90,y),(x+110,y)); dot(x+110,y)
    L((x+110,y),(x+110,y+75),(x-20,y+75),(x-20,y+25),(x,y+25))

GY=500
# title
T(60,68,"ServoDAC",size=24,weight="bold")
T(60,90,"closed-loop capacitor-charge DAC",size=13,fill="#555")

# M1
o.append('<rect x="250" y="200" width="150" height="260" fill="#fff" stroke="#111" stroke-width="2"/>')
T(240,196,"M1","end",weight="bold")
T(325,326,"Arduino","middle"); T(325,344,"Nano V3 / Uno","middle",size=12,fill="#555")
for y,s in((240,"A3"),(340,"A4"),(380,"A5")): T(258,y+5,s)
for y,s in((240,"A2"),(300,"D3"),(420,"D4")): T(392,y+5,s,"end")
L((325,200),(325,165)); vcc(325,165); T(333,192,"5V",size=11,fill="#555")
L((325,460),(325,GY)); gnd(325,GY); T(333,476,"GND",size=11,fill="#555")

# input + LCD
L((184,240),(250,240)); term(180,240); T(168,245,"Input","end"); note(168,261,"optional","end")
o.append('<rect x="60" y="315" width="110" height="90" fill="#fff" stroke="#111" stroke-width="2"/>')
T(60,307,"L1",weight="bold"); T(162,345,"SDA","end",size=12); T(162,385,"SCL","end",size=12)
L((170,340),(250,340)); L((170,380),(250,380))
T(60,423,"LCD1602, I²C",size=13); note(60,439,"optional; power not shown")

# charge path
L((400,300),(450,300)); rh(450,300); L((510,300),(815,300))
T(464,282,"R1","end",weight="bold"); T(472,282,"2.2 kΩ"); note(480,326,"charge: pulse HIGH, then hi-Z","middle")
# feedback
dot(570,300); L((570,300),(570,240),(400,240)); note(485,232,"feedback","middle")
# discharge
L((400,420),(440,420)); rh(440,420); L((500,420),(592,420))
T(470,402,"1 kΩ","middle"); note(470,446,"gate current limit","middle")
dot(636,300); L((636,300),(636,315)); rv(636,315); L((636,375),(636,396))
T(618,345,"RD","end",weight="bold")
T(618,362,"2.2 kΩ","end",size=13); note(618,381,"discharge","end")
# Q1 -- N-channel MOSFET, enlarged
L((592,388),(592,452),w=3)                               # gate bar
for a,b in ((388,404),(412,428),(436,452)):              # channel segments
    L((604,a),(604,b),w=3)
L((604,396),(636,396))                                   # drain to RD
L((604,444),(636,444),(636,GY))                          # source down to ground
L((604,420),(630,420),(630,444)); dot(636,444)           # body tie to source
o.append('<polygon points="605,420 617,414.5 617,425.5" fill="#111"/>')
gnd(636,GY)
T(652,404,"Q1",weight="bold"); T(652,424,"2N7000",size=13)
pin(586,412,"G","end"); pin(642,382,"D"); pin(642,466,"S")
# C1
dot(730,300); L((730,300),(730,450)); cap(730,450); L((730,460),(730,GY)); gnd(730,GY)
T(756,452,"C1",weight="bold"); T(756,469,"470 nF film",size=13)
# I1A
opamp(815,325,"I1A","3","2","1")
# post filter
L((925,325),(950,325)); rh(950,325); L((1010,325),(1090,325))
T(964,307,"R2","end",weight="bold"); T(972,307,"10 kΩ")
dot(1045,325); L((1045,325),(1045,450)); cap(1045,450); L((1045,460),(1045,GY)); gnd(1045,GY)
T(1020,452,"C2","end",weight="bold"); T(1020,469,"100 nF","end",size=13)
# I1B
opamp(1090,350,"I1B","5","6","7")
L((1200,350),(1236,350)); term(1240,350); T(1240,333,"Output","middle")

# I1 power unit
px=900
vcc(px,560); L((px,560),(px,590))
o.append(f'<rect x="{px-30}" y="590" width="60" height="50" fill="#fff" stroke="#111" stroke-width="2"/>')
T(px,620,"I1","middle",weight="bold")
L((px,640),(px,680)); gnd(px,680)
pin(px-6,587,"V+  8","end"); pin(px-6,656,"V−  4","end")
dot(px,575); dot(px,665)
L((px,575),(980,575),(980,615)); cap(980,615); L((980,625),(980,665),(px,665))
T(1006,617,"C3",weight="bold"); T(1006,634,"100 nF",size=13)
T(px-42,612,"LMC6482","end"); note(px-42,628,"supply, decoupled at the IC","end")

# notes
T(60,568,"Notes",weight="bold",size=13)
for i,s in enumerate(["D3 pulses HIGH to charge, then returns to high impedance, so R1 is",
                      "out of circuit while C1 holds or discharges. Pulse widths follow",
                      "t = −R1·C1·ln((5−Vt)/(5−Vs)) and t = −RD·C1·ln(Vt/Vs).",
                      "Charge, discharge and feedback pins are constructor parameters;",
                      "the feedback pin must be an analog input.",
                      "Input (A3) and L1 (A4/A5) are used only by the example sketches.",
                      "I1 pin numbers are for the 8-pin DIP/SOIC package."]):
    T(60,590+i*18,s,size=12,fill="#333")

svg=('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 720" width="1280" height="720" '
     'font-family="\'DejaVu Sans\', Helvetica, Arial, sans-serif">\n<rect width="1280" height="720" fill="#fff"/>\n'
     +"\n".join(o)+"\n</svg>\n")
open("schematic.svg","w").write(svg)
cairosvg.svg2png(bytestring=svg.encode(),write_to="schematic.png",output_width=1280,output_height=720)
cairosvg.svg2png(bytestring=svg.encode(),write_to="schematic@2x.png",output_width=2560,output_height=1440)
