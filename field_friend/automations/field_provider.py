import logging
import uuid
from typing import Any

import rosys
from geographiclib.geodesic import Geodesic
from rosys.geometry import Point

from .. import localization
from ..localization import GeoPoint, Gnss
from . import Field, FieldObstacle, Row


class FieldProvider(rosys.persistence.PersistentModule):

    def __init__(self, gnss: Gnss) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.field_provider')
        self.gnss = gnss
        self.fields: list[Field] = []

        self.FIELDS_CHANGED = rosys.event.Event()
        """The dict of fields has changed."""

        self.needs_backup: bool = False

        field = Field(id='Wiese_test_20241007', name='Wiese_Testfeld', points=[])
        for i, row_list in enumerate([[GeoPoint(lat=51.983183478278214, long=7.434241411038152), GeoPoint(lat=51.98319748684229, long=7.434292552530567)],
                                      [GeoPoint(lat=51.983179370626104, long=7.434244362155524),
                                       GeoPoint(lat=51.983193379188904, long=7.434295503644181)],
                                      [GeoPoint(lat=51.983175262974, long=7.434247313272896),
                                       GeoPoint(lat=51.98318927153552, long=7.434298354757795)],
                                      [GeoPoint(lat=51.98317115532166, long=7.4342502643886474),
                                       GeoPoint(lat=51.98318516388192, long=7.43430140586979)],
                                      [GeoPoint(lat=51.98316704766932, long=7.434253215504399),
                                       GeoPoint(lat=51.9831810562283, long=7.434304356981785)],
                                      [GeoPoint(lat=51.98316294001689, long=7.434256166619611),
                                       GeoPoint(lat=51.98317694857461, long=7.4343073080932385)],
                                      [GeoPoint(lat=51.98315883236441, long=7.434259117734283),
                                       GeoPoint(lat=51.983172840920844, long=7.434310259204153)],
                                      [GeoPoint(lat=51.98315472471183, long=7.434262068848415),
                                       GeoPoint(lat=51.98316873326699, long=7.434313210314528)],
                                      [GeoPoint(lat=51.98315061705919, long=7.434265019962007),
                                       GeoPoint(lat=51.98316462561307, long=7.434316161424363)],
                                      [GeoPoint(lat=51.98314650940645, long=7.43426797107506),
                                       GeoPoint(lat=51.98316051795908, long=7.434319112533658)]]):
            row = Row(id=f'row_{i}', name=f'row_{i}', points=list(row_list))
            field.rows.append(row)
        self.fields.append(field)

        field = Field(id='strasse', name='strasse', points=[])
        for i, row_list in enumerate([
            [GeoPoint(lat=51.982788618333345, long=7.434392561666668),
             GeoPoint(lat=51.982764325000005, long=7.434343425000002)],
            [GeoPoint(lat=51.98279212614165, long=7.434388013028723),
             GeoPoint(lat=51.98276783280644, long=7.4343388763606795)],
            [GeoPoint(lat=51.98279563394981, long=7.434383464390068),
             GeoPoint(lat=51.9827713406127, long=7.434334327720646)],
            [GeoPoint(lat=51.98279914175778, long=7.434378915750702),
             GeoPoint(lat=51.982774848418785, long=7.434329779079903)]
        ]):
            row = Row(id=f'row_{i}', name=f'row_{i}', points=list(row_list))
            field.rows.append(row)
        self.fields.append(field)

        field = Field(id='5.00m', name='5.00m', points=[])
        for i, row_list in enumerate([[GeoPoint(lat=51.98316780897856, long=7.434212), GeoPoint(lat=51.98321232423919, long=7.434212)],
                                      [GeoPoint(lat=51.983167808956075, long=7.4342847762085835),
                                       GeoPoint(lat=51.98321232421671, long=7.434284776280727)],
                                      [GeoPoint(lat=51.98316780888862, long=7.4343575524171674),
                                       GeoPoint(lat=51.98321232414926, long=7.434357552561454)],
                                      [GeoPoint(lat=51.9831678087762, long=7.4344303286257505),
                                       GeoPoint(lat=51.98321232403683, long=7.43443032884218)],
                                      [GeoPoint(lat=51.98316780861881, long=7.4345031048343335),
                                       GeoPoint(lat=51.98321232387944, long=7.434503105122906)],
                                      [GeoPoint(lat=51.98316780841646, long=7.434575881042917),
                                       GeoPoint(lat=51.98321232367709, long=7.434575881403632)],
                                      [GeoPoint(lat=51.983167808169135, long=7.434648657251498),
                                       GeoPoint(lat=51.98321232342976, long=7.434648657684356)],
                                      [GeoPoint(lat=51.983167807876846, long=7.434721433460079),
                                       GeoPoint(lat=51.98321232313747, long=7.4347214339650805)],
                                      [GeoPoint(lat=51.98316780753959, long=7.434794209668659),
                                       GeoPoint(lat=51.983212322800206, long=7.434794210245803)],
                                      [GeoPoint(lat=51.98316780715736, long=7.434866985877237),
                                       GeoPoint(lat=51.983212322417984, long=7.434866986526525)],
                                      [GeoPoint(lat=51.98316780673016, long=7.434939762085815), GeoPoint(lat=51.98321232199079, long=7.434939762807245)],]):
            row = Row(id=f'row_{i}', name=f'row_{i}', points=list(row_list))
            field.rows.append(row)
        self.fields.append(field)

        field = Field(id='0.45m', name='0.45m', points=[])
        for i, row_list in enumerate([[GeoPoint(lat=51.98316791930925, long=7.434212),
                                       GeoPoint(lat=51.98319451381729, long=7.434212)],
                                      [GeoPoint(lat=51.98316791930907, long=7.434218549858788),
                                       GeoPoint(lat=51.983194513817104, long=7.434218549862667)],
                                      [GeoPoint(lat=51.98316791930852, long=7.434225099717577),
                                       GeoPoint(lat=51.98319451381656, long=7.434225099725335)],
                                      [GeoPoint(lat=51.98316791930762, long=7.434231649576366),
                                       GeoPoint(lat=51.983194513815654, long=7.4342316495880025)],
                                      [GeoPoint(lat=51.98316791930634, long=7.434238199435154),
                                       GeoPoint(lat=51.983194513814375, long=7.43423819945067)],
                                      [GeoPoint(lat=51.9831679193047, long=7.434244749293943),
                                       GeoPoint(lat=51.983194513812734, long=7.434244749313338)],
                                      [GeoPoint(lat=51.983167919302694, long=7.434251299152732),
                                       GeoPoint(lat=51.98319451381073, long=7.4342512991760055)],
                                      [GeoPoint(lat=51.983167919300335, long=7.43425784901152),
                                       GeoPoint(lat=51.983194513808364, long=7.434257849038673)],
                                      [GeoPoint(lat=51.9831679192976, long=7.434264398870309),
                                       GeoPoint(lat=51.983194513805635, long=7.434264398901341)],
                                      [GeoPoint(lat=51.9831679192945, long=7.434270948729098),
                                       GeoPoint(lat=51.98319451380254, long=7.434270948764008)],]):
            row = Row(id=f'row_{i}', name=f'row_{i}', points=list(row_list))
            field.rows.append(row)
        self.fields.append(field)
        self.FIELDS_CHANGED.emit()

    def get_field(self, id_: str | None) -> Field | None:
        for field in self.fields:
            if field.id == id_:
                return field
        return None

    def backup(self) -> dict:
        return {
            'fields': rosys.persistence.to_dict(self.fields),
        }

    def restore(self, data: dict[str, Any]) -> None:
        fields_data = data.get('fields', [])
        rosys.persistence.replace_list(self.fields, Field, fields_data)

        # NOTE we had some changes in persistence; this code transforms old to new format
        for i, f in enumerate(self.fields):
            outline = fields_data[i].get('outline_wgs84', [])
            for coords in outline:
                f.points.append(GeoPoint(lat=coords[0], long=coords[1]))
            rows = fields_data[i].get('rows', [])
            for j, row in enumerate(rows):
                for point in row.get('points_wgs84', []):
                    f.rows[j].points.append(GeoPoint(lat=point[0], long=point[1]))

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELDS_CHANGED.emit()

    def create_field(self, points: list[GeoPoint] | None = None) -> Field:
        if points is None:
            points = []
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'field_{len(self.fields)+1}', points=points)
        self.fields.append(field)
        self.invalidate()
        return field

    def remove_field(self, field: Field) -> None:
        self.fields.remove(field)
        self.FIELDS_CHANGED.emit()
        self.invalidate()

    def clear_fields(self) -> None:
        self.fields.clear()
        self.FIELDS_CHANGED.emit()
        self.invalidate()

    def create_obstacle(self, field: Field, points: list[GeoPoint] | None = None) -> FieldObstacle:
        if points is None:
            points = []
        obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}', name=f'obstacle_{len(field.obstacles)+1}', points=points)
        field.obstacles.append(obstacle)
        self.invalidate()
        return obstacle

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        field.obstacles.remove(obstacle)
        self.invalidate()

    def create_row(self, field: Field, points: list[GeoPoint] | None = None) -> Row:
        if points is None:
            points = []
        row = Row(id=f'{str(uuid.uuid4())}', name=f'row_{len(field.rows)+1}', points=points)
        field.rows.append(row)
        self.invalidate()
        return row

    def remove_row(self, field: Field, row: Row) -> None:
        field.rows.remove(row)
        self.invalidate()

    def is_polygon(self, field: Field) -> bool:
        try:
            polygon = field.shapely_polygon
            return polygon.is_valid and polygon.geom_type == 'Polygon'
        except Exception:
            return False

    def sort_rows(self, field: Field) -> None:
        # TODO currently the function does only sort even fields where rows have the same length
        # the function need to be extended for more special cases
        if len(field.rows) <= 1:
            rosys.notify(f'There are not enough rows that can be sorted.', type='warning')
            return

        for row in field.rows:
            if len(row.points) < 1:
                rosys.notify(f'Row {row.name} has to few points. Sorting not possible.', type='warning')
                return

        def get_centroid(row: Row) -> Point:
            return row.shapely_line.centroid

        reference_centroid = get_centroid(field.rows[0])

        def get_distance(row: Row, direction: str):
            row_centroid = get_centroid(row)
            distance = Geodesic.WGS84.Inverse(reference_centroid.x, reference_centroid.y,
                                              row_centroid.x, row_centroid.y)["s12"]
            if direction == "x" and reference_centroid.x > row_centroid.x:
                distance *= -1
            elif direction == "y" and reference_centroid.y > row_centroid.y:
                distance *= -1
            print(f'{row.name} has a dist of {distance}')

            return distance

        field_index = self.fields.index(field)
        p = field.rows[-1].points[0]
        direction = "x" if abs(reference_centroid.x - p.lat) > abs(reference_centroid.y - p.long) else "y"
        self.fields[field_index].rows = sorted(field.rows, key=lambda row: get_distance(row, direction=direction))
        self.FIELDS_CHANGED.emit()

    async def add_field_point(self, field: Field, point: GeoPoint | None = None, new_point: GeoPoint | None = None) -> None:
        assert self.gnss.current is not None
        positioning = self.gnss.current.location
        if positioning is None or positioning.lat == 0 or positioning.long == 0:
            rosys.notify("No GNSS position.")
            return
        if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            rosys.notify("GNSS position is not accurate enough.")
            return
        new_point = positioning
        if point is not None:
            index = field.points.index(point)
            field.points[index] = new_point
        else:
            field.points.append(new_point)
        self.invalidate()

    def remove_field_point(self, field: Field, point: GeoPoint | None = None) -> None:
        if point is not None:
            index = field.points.index(point)
            del field.points[index]
        elif field.points:
            del field.points[-1]
        self.invalidate()

    def add_obstacle_point(self, field: Field, obstacle: FieldObstacle, point: GeoPoint | None = None, new_point: GeoPoint | None = None) -> None:
        if new_point is None:
            assert self.gnss.current is not None
            positioning = self.gnss.current.location
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
                rosys.notify("GNSS position is not accurate enough.")
                return
            new_point = positioning
        if point is not None:
            index = obstacle.points.index(point)
            obstacle.points[index] = new_point
        else:
            obstacle.points.append(new_point)
        self.invalidate()

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: GeoPoint | None = None) -> None:
        if obstacle.points:
            if point is not None:
                index = obstacle.points.index(point)
                del obstacle.points[index]
            else:
                del obstacle.points[-1]
            self.invalidate()

    def add_row_point(self, field: Field, row: Row, point: GeoPoint | None = None, new_point: GeoPoint | None = None) -> None:
        if new_point is None:
            assert self.gnss.current is not None
            positioning = self.gnss.current.location
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
                rosys.notify("GNSS position is not accurate enough.")
                return
            new_point = positioning
        if point is not None:
            index = row.points.index(point)
            row.points[index] = new_point
        else:
            row.points.append(new_point)
        self.invalidate()

    def remove_row_point(self, row: Row, point: GeoPoint | None = None) -> None:
        if row.points:
            if point is not None:
                index = row.points.index(point)
                del row.points[index]
            else:
                del row.points[-1]
            self.invalidate()

    def move_row(self, field: Field, row: Row, next: bool = False) -> None:
        index = field.rows.index(row)
        if next:
            if index == len(field.rows)-1:
                field.rows[index], field.rows[0] = field.rows[0], field.rows[index]
            else:
                field.rows[index], field.rows[index+1] = field.rows[index+1], field.rows[index]
        else:
            field.rows[index], field.rows[index-1] = field.rows[index-1], field.rows[index]
        self.invalidate()
