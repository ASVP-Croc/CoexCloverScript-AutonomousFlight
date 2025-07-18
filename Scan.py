#!/usr/bin/env python3
import rospy
from clover import srv
from std_srvs.srv import Trigger
from shapely.geometry import Polygon, Point
from shapely.errors import TopologicalError
import numpy as np
import math
import sys

previous_position = [0, 0, 0]
first_navigation = True
FRAME_ID = 'map'

# ────────────── GEOMETRIA & SCANSIONE ──────────────
def order_vertices_to_avoid_intersection(vertices):
    cx = sum(v[0] for v in vertices) / len(vertices)
    cy = sum(v[1] for v in vertices)
    def angle_from_center(p): return math.atan2(p[1] - cy, p[0] - cx)
    return sorted(vertices, key=angle_from_center)

def generate_hex_grid_with_buffer_and_filter(polygon, r, threshold=0.1):
    row_points = []
    removed_points = []
    minx, miny, maxx, maxy = polygon.bounds
    dx = 1.5 * r
    dy = np.sqrt(3) * r
    y = miny - r
    row = 0
    while y <= maxy + r:
        x_offset = 0.75 * r if row % 2 else 0
        x = minx - r + x_offset
        current_row = []
        while x <= maxx + r:
            point = Point(x, y)
            circle = point.buffer(r)
            try:
                intersection = circle.intersection(polygon)
                if not intersection.is_empty and intersection.area / circle.area >= threshold:
                    current_row.append((x, y))
                else:
                    removed_points.append((x, y))
            except TopologicalError:
                pass
            x += dx
        if current_row:
            row_points.append(current_row)
        y += dy
        row += 1
    return row_points, removed_points

def optimize_path_layered_with_final_interleaved(row_points):
    optimized_path = []
    skipped_points = []
    total_rows = len(row_points)
    interleaved_handled = False
    normal_rows = row_points

    if total_rows % 2 == 1 and total_rows >= 2:
        interleaved_rows = [row_points[-2], row_points[-1]]
        normal_rows = row_points[:-2]
        interleaved_handled = True

    for idx, row in enumerate(normal_rows):
        if not row:
            continue
        sorted_row = sorted(row, key=lambda p: p[0])
        skipped = sorted_row[0]
        main_points = sorted_row[1:]
        main_order = main_points if idx % 2 == 0 else list(reversed(main_points))
        optimized_path.extend(main_order)
        skipped_points.insert(0, skipped)

    if interleaved_handled:
        row1 = sorted(interleaved_rows[0], key=lambda p: p[0])
        row2 = sorted(interleaved_rows[1], key=lambda p: p[0])
        skipped1 = row1[0]
        main1 = row1[1:]
        main2 = row2
        interleaved = []
        for i in range(min(len(main1), len(main2))):
            interleaved.append(main1[-(i + 1)])
            interleaved.append(main2[-(i + 1)])
        if len(main1) > len(main2):
            interleaved.extend(reversed(main1[:len(main1) - len(main2)]))
        elif len(main2) > len(main1):
            interleaved.extend(reversed(main2[:len(main2) - len(main1)]))
        optimized_path.extend(interleaved)
        skipped_points.insert(0, skipped1)

    optimized_path.extend(skipped_points)
    return optimized_path

# ────────────── NAVIGAZIONE ──────────────
def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=0.0, speed=0.2, hover_time=2.0, frame_id=FRAME_ID):
    global previous_position, first_navigation

    rospy.wait_for_service('navigate')
    rospy.wait_for_service('get_telemetry')

    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

    try:
        res = navigate(
            x=x, y=y, z=z, yaw=yaw,
            speed=speed, frame_id=frame_id,
            auto_arm=first_navigation
        )
        first_navigation = False

        rospy.loginfo(f"[NAVIGATE] Muovo a ({x:.2f}, {y:.2f}, {z:.2f}) a {speed:.2f} m/s")
        dist = np.linalg.norm(np.array([x, y, z]) - np.array(previous_position))
        previous_position = [x, y, z]

        timeout = dist / speed + hover_time + 1.0
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < timeout:
            telem = get_telemetry(frame_id=frame_id)
            err = np.linalg.norm(np.array([telem.x, telem.y, telem.z]) - np.array([x, y, z]))
            if err < 0.15:
                break
            rospy.sleep(0.3)

        telem = get_telemetry(frame_id=frame_id)
        rospy.loginfo(f"[Telemetry] Posizione: ({telem.x:.2f}, {telem.y:.2f}, {telem.z:.2f}) Modalità: {telem.mode}")
        return res
    except rospy.ServiceException as e:
        rospy.logerr(f"Errore nella navigazione: {e}")

# ────────────── DECOLLO ──────────────
def takeoff(altitude, speed, hover_time):
    rospy.wait_for_service('navigate')
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    global first_navigation
    rospy.loginfo(f"[TAKEOFF] Decollo a {altitude:.2f} m...")
    try:
        navigate(x=0.0, y=0.0, z=altitude, yaw=0.0, speed=speed, frame_id='body', auto_arm=first_navigation)
        first_navigation = False
        rospy.sleep(altitude / speed + hover_time)
    except rospy.ServiceException as e:
        rospy.logerr(f"Errore nel decollo: {e}")
        safe_shutdown()

# ────────────── INPUT & CALCOLI ──────────────
def get_coordinates():
    print("Inserisci i 4 vertici (es: 4.0,3.0):")
    raw_vertices = []
    for i in range(4):
        while True:
            v = input(f"Vertice {i+1}: ")
            try:
                x, y = map(float, v.split(','))
                raw_vertices.append((x, y))
                break
            except Exception:
                print("Formato non valido. Riprova.")
    return raw_vertices

def calcola_tempo_totale(percorso, velocita, hover, altitudine):
    punti = [(0.0, 0.0, 0.0)] + [(x, y, altitudine) for x, y in percorso] + [(0.0, 0.0, altitudine)]
    tempo = 0.0
    for i in range(len(punti)-1):
        p1 = np.array(punti[i])
        p2 = np.array(punti[i+1])
        distanza = np.linalg.norm(p2 - p1)
        tempo += distanza / velocita
    tempo += len(percorso) * hover
    return tempo

# ────────────── SHUTDOWN SICURO ──────────────
def safe_shutdown():
    if not rospy.is_shutdown():
        try:
            rospy.logwarn("Eseguo atterraggio di emergenza (shutdown)...")
            rospy.wait_for_service('land', timeout=5)
            land = rospy.ServiceProxy('land', Trigger)
            land()
            rospy.loginfo("Atterraggio completato con successo.")
        except Exception as e:
            rospy.logerr(f"Errore nell'atterraggio d'emergenza: {e}")
    else:
        rospy.logwarn("ROS è già in shutdown. Non posso eseguire atterraggio.")

# ────────────── MAIN ──────────────
def main():
    rospy.init_node('hex_flight')
    rospy.on_shutdown(safe_shutdown)

    raw_vertices = get_coordinates()
    raggio = float(input("Raggio di scansione (es: 1.0): "))
    altitude = float(input("Altezza volo (es: 2.0): "))
    speed = float(input("Velocità volo (es: 0.5): "))
    hover_time = float(input("Tempo stazionamento (es: 3.0): "))

    ordered_vertices = order_vertices_to_avoid_intersection(raw_vertices)
    poligono = Polygon(ordered_vertices)
    if not poligono.is_valid:
        print("[!] Poligono non valido. Correzione...")
        poligono = Polygon(order_vertices_to_avoid_intersection(raw_vertices))

    righe_punti, _ = generate_hex_grid_with_buffer_and_filter(poligono, raggio)
    percorso = optimize_path_layered_with_final_interleaved(righe_punti)

    tempo_totale = calcola_tempo_totale(percorso, speed, hover_time, altitude)
    print(f"Totale punti: {len(percorso)} - Durata stimata: {tempo_totale:.1f} s\n")

    try:
        takeoff(altitude, speed, hover_time)

        for x, y in percorso:
            if rospy.is_shutdown():
                break
            navigate_wait(x=x, y=y, z=altitude, speed=speed, hover_time=hover_time)

        if not rospy.is_shutdown():
            navigate_wait(x=0.0, y=0.0, z=altitude, speed=speed, hover_time=hover_time)

            rospy.wait_for_service('land')
            land = rospy.ServiceProxy('land', Trigger)
            land()
            rospy.loginfo("Atterraggio finale completato.")

            rospy.wait_for_service('simple_offboard/release')
            release = rospy.ServiceProxy('simple_offboard/release', Trigger)
            release()
            rospy.loginfo("Controllo OFFBOARD rilasciato.")

    finally:
        safe_shutdown()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Script interrotto da ROS. Tentativo di atterraggio...")
        safe_shutdown()
        pass  # sopprime l'eccezione, non mostra stack trace