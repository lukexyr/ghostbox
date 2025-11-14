import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import mean_squared_error, r2_score
import joblib
import matplotlib.pyplot as plt

class IMUCalibrationModel:
    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.model = None
        self.scaler = None
        self.sensor_names = ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz']
        
    def load_and_align_data(self):
        """Load CSV and align iPhone and Nano data by timestamp"""
        df = pd.read_csv(self.csv_file)
        
        # Separate data sources
        nano_df = df[df['source'] == 'nano'].copy()
        iphone_df = df[df['source'] == 'iphone'].copy()
        
        print(f"Nano samples: {len(nano_df)}")
        print(f"iPhone samples: {len(iphone_df)}")
        
        # Align data by finding closest timestamps
        aligned_data = []
        
        for _, nano_row in nano_df.iterrows():
            nano_time = nano_row['timestamp']
            
            # Find closest iPhone timestamp
            time_diffs = np.abs(iphone_df['timestamp'] - nano_time)
            closest_idx = time_diffs.idxmin()
            
            # Only align if timestamps are within 100ms
            if time_diffs[closest_idx] < 0.1:
                iphone_row = iphone_df.loc[closest_idx]
                
                # Nano data as input
                nano_values = [nano_row[s] for s in self.sensor_names]
                # iPhone data as target
                iphone_values = [iphone_row[s] for s in self.sensor_names]
                
                aligned_data.append({
                    'nano': nano_values,
                    'iphone': iphone_values,
                    'time_diff': time_diffs[closest_idx]
                })
        
        print(f"Aligned pairs: {len(aligned_data)}")
        print(f"Average time difference: {np.mean([d['time_diff'] for d in aligned_data])*1000:.2f}ms")
        
        # Convert to arrays
        X = np.array([d['nano'] for d in aligned_data])
        y = np.array([d['iphone'] for d in aligned_data])
        
        return X, y
    
    def train_model(self, X, y):
        """Train neural network to calibrate Nano data"""
        print("\n" + "="*50)
        print("Training Calibration Model")
        print("="*50)
        
        # Split data
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=42
        )
        
        # Scale features
        self.scaler = StandardScaler()
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_test_scaled = self.scaler.transform(X_test)
        
        # Train model with multiple hidden layers
        self.model = MLPRegressor(
            hidden_layer_sizes=(128, 64, 32),
            activation='relu',
            solver='adam',
            max_iter=1000,
            learning_rate_init=0.001,
            early_stopping=True,
            validation_fraction=0.1,
            n_iter_no_change=20,
            random_state=42,
            verbose=True
        )
        
        print("\nTraining neural network...")
        self.model.fit(X_train_scaled, y_train)
        
        # Evaluate
        train_pred = self.model.predict(X_train_scaled)
        test_pred = self.model.predict(X_test_scaled)
        
        train_r2 = r2_score(y_train, train_pred)
        test_r2 = r2_score(y_test, test_pred)
        train_mse = mean_squared_error(y_train, train_pred)
        test_mse = mean_squared_error(y_test, test_pred)
        
        print("\n" + "="*50)
        print("Model Performance")
        print("="*50)
        print(f"Training R² Score: {train_r2:.4f}")
        print(f"Testing R² Score: {test_r2:.4f}")
        print(f"Training MSE: {train_mse:.6f}")
        print(f"Testing MSE: {test_mse:.6f}")
        
        # Per-sensor performance
        print("\nPer-Sensor R² Scores (Test Set):")
        for i, sensor in enumerate(self.sensor_names):
            sensor_r2 = r2_score(y_test[:, i], test_pred[:, i])
            print(f"  {sensor}: {sensor_r2:.4f}")
        
        return X_test_scaled, y_test, test_pred
    
    def visualize_results(self, X_test, y_test, y_pred):
        """Visualize calibration results"""
        fig, axes = plt.subplots(3, 3, figsize=(15, 12))
        fig.suptitle('IMU Calibration Results: Predicted vs Actual', fontsize=16)
        
        for i, sensor in enumerate(self.sensor_names):
            row = i // 3
            col = i % 3
            ax = axes[row, col]
            
            # Scatter plot
            ax.scatter(y_test[:, i], y_pred[:, i], alpha=0.5, s=10)
            
            # Perfect prediction line
            min_val = min(y_test[:, i].min(), y_pred[:, i].min())
            max_val = max(y_test[:, i].max(), y_pred[:, i].max())
            ax.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='Perfect')
            
            ax.set_xlabel(f'Actual {sensor}')
            ax.set_ylabel(f'Predicted {sensor}')
            ax.set_title(f'{sensor} (R²={r2_score(y_test[:, i], y_pred[:, i]):.3f})')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('calibration_results.png', dpi=150)
        print("\n✓ Visualization saved to 'calibration_results.png'")
        plt.show()
    
    def save_model(self, filename='imu_calibration_model.pkl'):
        """Save trained model and scaler"""
        if self.model is None or self.scaler is None:
            print("No model to save!")
            return
        
        joblib.dump({
            'model': self.model,
            'scaler': self.scaler,
            'sensor_names': self.sensor_names
        }, filename)
        
        print(f"\n✓ Model saved to '{filename}'")
    
    def calibrate_nano_data(self, nano_readings):
        """
        Calibrate new Arduino Nano readings
        
        Args:
            nano_readings: List or array of 9 values [ax,ay,az,gx,gy,gz,mx,my,mz]
        
        Returns:
            Calibrated readings as numpy array
        """
        if self.model is None or self.scaler is None:
            raise ValueError("Model not trained yet!")
        
        # Ensure input is 2D array
        nano_readings = np.array(nano_readings).reshape(1, -1)
        
        # Scale and predict
        scaled = self.scaler.transform(nano_readings)
        calibrated = self.model.predict(scaled)
        
        return calibrated[0]
    
    def run_full_pipeline(self):
        """Execute complete training pipeline"""
        print("Loading and aligning data...")
        X, y = self.load_and_align_data()
        
        if len(X) < 100:
            print("Warning: Very few aligned samples. Consider recording more data.")
        
        X_test, y_test, y_pred = self.train_model(X, y)
        self.visualize_results(X_test, y_test, y_pred)
        self.save_model()
        
        print("\n" + "="*50)
        print("Training Complete!")
        print("="*50)
        print("You can now use the model to calibrate Arduino Nano data in real-time.")


def load_model(filename='imu_calibration_model.pkl'):
    """Load a trained model for inference"""
    data = joblib.load(filename)
    calibrator = IMUCalibrationModel(None)
    calibrator.model = data['model']
    calibrator.scaler = data['scaler']
    calibrator.sensor_names = data['sensor_names']
    return calibrator


# Example usage
if __name__ == "__main__":
    # Train the model
    csv_file = "imu_test_data_20251113_185925.csv"  # Update with your CSV filename
    
    calibrator = IMUCalibrationModel(csv_file)
    calibrator.run_full_pipeline()
    
    # Example: Calibrate new readings
    print("\n" + "="*50)
    print("Example: Calibrating New Readings")
    print("="*50)
    
    # Simulated Nano readings
    nano_readings = [0.1, 0.2, 9.8, 0.01, -0.02, 0.03, 30.5, -5.2, 40.1]
    
    calibrated = calibrator.calibrate_nano_data(nano_readings)
    
    print("\nOriginal Nano readings:")
    for i, sensor in enumerate(calibrator.sensor_names):
        print(f"  {sensor}: {nano_readings[i]:.4f}")
    
    print("\nCalibrated readings:")
    for i, sensor in enumerate(calibrator.sensor_names):
        print(f"  {sensor}: {calibrated[i]:.4f}")