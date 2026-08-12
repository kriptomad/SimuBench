use std::collections::HashMap;
use std::time::Instant;

#[derive(Debug, Clone)]
struct TokenBucket {
    tokens: f64,
    capacity: f64,
    refill_per_sec: f64,
    last_refill: Instant,
}

impl TokenBucket {
    fn new(rate_per_sec: u32, burst: u32) -> Self {
        let cap = burst.max(1) as f64;
        Self {
            tokens: cap,
            capacity: cap,
            refill_per_sec: rate_per_sec.max(1) as f64,
            last_refill: Instant::now(),
        }
    }

    fn try_take(&mut self, amount: f64) -> bool {
        self.refill();
        if self.tokens >= amount {
            self.tokens -= amount;
            true
        } else {
            false
        }
    }

    fn refill(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_refill).as_secs_f64();
        self.last_refill = now;
        self.tokens = (self.tokens + elapsed * self.refill_per_sec).min(self.capacity);
    }
}

#[derive(Debug, Clone)]
pub struct TwoLevelRateLimiter {
    global: TokenBucket,
    per_id_rate: u32,
    per_id: HashMap<u32, TokenBucket>,
}

impl TwoLevelRateLimiter {
    pub fn new(global_per_sec: u32, per_id_per_sec: u32) -> Self {
        Self {
            global: TokenBucket::new(global_per_sec, global_per_sec),
            per_id_rate: per_id_per_sec.max(1),
            per_id: HashMap::new(),
        }
    }

    pub fn check_can(&mut self, can_id: u32) -> bool {
        if !self.global.try_take(1.0) {
            return false;
        }

        let per = self
            .per_id
            .entry(can_id)
            .or_insert_with(|| TokenBucket::new(self.per_id_rate, self.per_id_rate));

        per.try_take(1.0)
    }

    pub fn check_serial(&mut self) -> bool {
        self.global.try_take(1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limiter_blocks_when_empty() {
        let mut lim = TwoLevelRateLimiter::new(1, 1);
        assert!(lim.check_can(0x100));
        assert!(!lim.check_can(0x100));
    }
}
