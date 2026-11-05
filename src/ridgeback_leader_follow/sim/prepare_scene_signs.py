"""Draw original wayfinding panels used as architectural sign textures."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont

PANELS={
 'sign_reception':('RECEPTION','INFORMATION  /  VISITORS',(.06,.15,.2)),
 'sign_office':('LABORATORIES','OFFICES  01 - 06',(.06,.15,.2)),
 'sign_exit':('EXIT  >','KEEP THIS ROUTE CLEAR',(.04,.3,.16)),
 'sign_loading':('LOADING  02','WAREHOUSE  /  DISPATCH',(.15,.18,.19)),
 'sign_safety':('PEDESTRIAN ROUTE','STAY INSIDE THE MARKED AISLE',(.78,.48,.03)),
 'sign_courtyard':('SCIENCE COURTYARD','RECEPTION  >     GARDENS  <',(.12,.24,.18)),
}


def main():
    root=Path(__file__).resolve().parent/'assets/realistic/surfaces'
    for name,(title,subtitle,color) in PANELS.items():
        image=Image.new('RGB',(1024,320),tuple(round(v*255) for v in color));draw=ImageDraw.Draw(image)
        font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        draw.rounded_rectangle((10,10,1014,310),radius=14,outline=(220,225,216),width=3)
        draw.text((48,65),title,font=ImageFont.truetype(font,57 if len(title)>18 else 72),fill=(242,244,235))
        draw.text((50,200),subtitle,font=ImageFont.truetype(font,28),fill=(219,225,216))
        output=root/name;output.mkdir(parents=True,exist_ok=True);image.save(output/'Diffuse.jpg',quality=95)


if __name__=='__main__':main()
