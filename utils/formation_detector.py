import numpy as np

from itertools import permutations
from scipy.spatial import procrustes

class FormationDetection:
    def __init__(self, disparity_threshold=0.1):
        self.disparity_threshold = disparity_threshold
        
        self.formations = [
            {
                "name": "Üçgen",
                "positions": np.array([[0, 0], [1, 0], [0.5, np.sqrt(3)/2]]),
            },
            {
                "name": "Çizgi",
                "positions": np.array([[0, 0], [1, 0], [2, 0]]),
            },
            {
                "name": "Ok Başı",
                "positions": np.array([[0, 1], [1, 0], [2, 1]]),
            }
        ]
        
    def _calculate_disparity(self, formation1, formation2):
        # Formasyonları normalize etmek için Procrustes analizi kullan
        mtx1, mtx2, disparity = procrustes(formation1, formation2)
        
        # Permutasyonları deneyerek en düşük dispariteyi bul
        min_disparity = float('inf')
        best_perm = None
        
        for perm in permutations(range(len(formation2))):
            permuted_formation = mtx1[list(perm)]
            current_disparity = np.linalg.norm(permuted_formation - mtx2)
            if current_disparity < min_disparity:
                min_disparity = current_disparity
                best_perm = perm
        
        return min_disparity, best_perm
    
    def detect_formation(self, current_formation):
        match = None
        disparities = []
        for formation in self.formations:
            disparity, perm = self._calculate_disparity(formation["positions"], current_formation)
            # Eğer disparite eşik değerinden düşükse, formasyon eşleşti
            if disparity < self.disparity_threshold:
                disparities.append({
                    "name": formation["name"],
                    "disparity": disparity,
                    "permutation": perm
                })
        for d in disparities:
            # print(f"Formasyon: {d['name']}, Farklılık Oranı: %{round(d['disparity']*100,2)}, Dizilim: {d['permutation']}")
            if match is None or d['disparity'] < match['disparity']:
                match = d
        return match

# Example usage:
if __name__ == "__main__":
    current_formation = np.array([[0, 3], [1, 1], [4, 3]])
    
    detector = FormationDetection(0.5)
    result = detector.detect_formation(current_formation)
    
    if result:
        print(f"Formasyon Algılandı: {result['name']}\nFarklılık Oranı: %{round(result['disparity']*100,2)}")
    else:
        print("No matching formation detected.")